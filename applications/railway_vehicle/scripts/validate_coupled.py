"""Run reproducible dt/modal/FEM studies using only the Python standard library.

Errors compare the complete contact force, vehicle acceleration and rail response
histories, not just displacement peaks. A failed run stays failed in the report.
"""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import subprocess
import time


def read_csv(path):
    with path.open(encoding="utf-8", newline="") as stream:
        return [{key: float(value) for key, value in row.items()} for row in csv.DictReader(stream)]


def compare(candidate, reference):
    reference_times = [row["time_s"] for row in reference]
    keys = [key for key in reference[0] if key.endswith("_force_N") or key.endswith("_acceleration")
            or key.endswith("_rail_z_m")]
    result = {}
    for key in keys:
        errors, expected = [], []
        for row in candidate:
            t = row["time_s"]
            index = min(max(1, bisect.bisect_left(reference_times, t)), len(reference)-1)
            left, right = reference[index-1], reference[index]
            fraction = (t-left["time_s"])/(right["time_s"]-left["time_s"])
            value = left[key] + fraction*(right[key]-left[key])
            expected.append(value)
            errors.append(row[key]-value)
        rmse = math.sqrt(sum(e*e for e in errors)/len(errors))
        rms = math.sqrt(sum(v*v for v in expected)/len(expected))
        result[key] = {"rmse": rmse, "nrmse": rmse/rms if rms > 1e-12 else None,
                       "max_abs_error": max(abs(e) for e in errors), "reference_rms": rms}
    return result


def event_comparison(candidate, reference):
    def groups(rows):
        result = {}
        for row in rows:
            key = (int(row["wheel_id"]), int(row["rail_id"]), int(row["to_contact"]))
            result.setdefault(key, []).append(row["time_s"])
        return result
    a, b = groups(candidate), groups(reference)
    same_counts = all(len(a.get(key, [])) == len(b.get(key, [])) for key in a.keys() | b.keys())
    errors = [abs(x-y) for key in a.keys() & b.keys() for x, y in zip(a[key], b[key])]
    return {"candidate_count": len(candidate), "reference_count": len(reference),
            "matching_event_counts": same_counts,
            "max_event_time_error_s": max(errors) if errors and same_counts else None,
            "note": "Events are sampled at accepted timestep endpoints; no substep root localization."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=0.2)
    parser.add_argument("--dt", type=float, default=1e-4, help="Fixed timestep for modal/FEM comparisons")
    parser.add_argument("--reference-modes", type=int, default=320)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0 or not math.isfinite(args.dt) or args.dt <= 0:
        parser.error("duration and dt must be positive and finite")
    if args.reference_modes <= 80 or args.reference_modes > 1000:
        parser.error("reference-modes must be in 81..1000")
    exe, source = args.exe.resolve(), args.config.resolve()
    base = json.loads(source.read_text(encoding="utf-8-sig"))
    base["vehicle_config"] = str((source.parent / base["vehicle_config"]).resolve())
    base["duration_s"] = args.duration
    base["dt_s"] = args.dt
    base.setdefault("rail", {})["modes"] = args.reference_modes
    args.output.mkdir(parents=True, exist_ok=False)
    runs = {}

    def run(name, *, dt=None, modes=None, elements=None, phase=None, backend="modal"):
        configuration = json.loads(json.dumps(base))
        configuration["backend"] = backend
        configuration["dt_s"] = args.dt if dt is None else dt
        if phase is not None:
            configuration["max_phase_increment"] = phase
        if modes is not None:
            configuration["rail"]["modes"] = modes
        if elements is not None:
            configuration["rail"]["elements"] = elements
        path = args.output / (name + ".json")
        path.write_text(json.dumps(configuration, indent=2), encoding="utf-8")
        out = args.output / name
        start = time.perf_counter()
        process = subprocess.run([str(exe), "--config", str(path), "--output", str(out)],
                                 capture_output=True, text=True, errors="replace", check=False)
        record = {"returncode": process.returncode, "subprocess_wall_seconds": time.perf_counter()-start,
                  "dt_s": configuration["dt_s"], "modes": configuration["rail"]["modes"],
                  "elements": configuration["rail"].get("elements", 120), "backend": backend,
                  "stdout": process.stdout.strip(), "stderr": process.stderr.strip()}
        if process.returncode == 0:
            record["summary"] = dict(line.split("=", 1) for line in (out/"summary.txt").read_text().splitlines() if "=" in line)
        runs[name] = record
        print(name, "PASS" if process.returncode == 0 else "FAILED", flush=True)
        return record

    run("reference")
    run("phase_half", phase=base.get("max_phase_increment", 0.2)/2)
    # Reference refinement is reported separately; no assumption that 320 modes is exact.
    if args.reference_modes*2 <= 1000:
        run("reference_refined", modes=args.reference_modes*2)
    for dt in [1e-2, 1e-3, 5e-4, 1e-4]:
        run("dt_" + str(dt), dt=dt)
    for modes in [5, 10, 20, 40, 80]:
        run("modes_" + str(modes), modes=modes)
    for elements in [120, 240]:
        run("fem_" + str(elements), elements=elements, backend="fem")

    reference = None
    if runs["reference"]["returncode"] == 0:
        reference = read_csv(args.output / "reference/response.csv")
        reference_events = read_csv(args.output / "reference/events.csv")
        for name, record in runs.items():
            if name == "reference" or record["returncode"]:
                continue
            record["metrics"] = compare(read_csv(args.output/name/"response.csv"), reference)
            record["events"] = event_comparison(read_csv(args.output/name/"events.csv"), reference_events)
            forces = [value["nrmse"] for key, value in record["metrics"].items() if key.endswith("_force_N")]
            record["max_contact_force_nrmse"] = max(forces) if all(x is not None for x in forces) else None
            record["force_below_1_percent"] = all(x is not None and x < 0.01 for x in forces)
    report = {"reference": "reference", "runs": runs,
              "limitations": ["Numerical consistency study, not experimental validation.",
                              "FEM and analytic sine modes share the exact oscillator time propagator; spatial discretizations differ.",
                              "FEM retains all free element DOFs. Timing includes CSV output; process clock semantics depend on OS.",
                              "dt_s is the output/macro step; frequency-controlled internal steps are reported separately.",
                              "A 1% contact-force threshold does not imply acceleration or event-time convergence."]}
    (args.output/"validation.json").write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    lines = ["# Coupled numerical validation", "", "Reference: %s modes, dt=%g s, duration=%g s." %
             (args.reference_modes, args.dt, args.duration), "",
             "| Run | Status | Max force NRMSE | Force <1% | Internal steps | Wall time (s) |",
             "|---|---|---:|---|---:|---:|"]
    for name, record in runs.items():
        error = record.get("max_contact_force_nrmse")
        lines.append("| %s | %s | %s | %s | %s | %.3f |" %
                     (name, "PASS" if record["returncode"] == 0 else "FAILED",
                      "—" if error is None else "%.4f%%" % (100*error),
                      str(record.get("force_below_1_percent", "—")), record.get("summary", {}).get("internal_steps", "—"), record["subprocess_wall_seconds"]))
    lines.extend(["", "Per-channel RMSE, NRMSE, maximum error, event comparisons and actual solver timings are in validation.json.",
                  "", *report["limitations"]])
    (args.output/"validation.md").write_text("\n".join(lines)+"\n", encoding="utf-8")
    return 0 if reference is not None and all(r["returncode"] == 0 for r in runs.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
