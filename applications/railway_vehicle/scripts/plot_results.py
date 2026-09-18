"""Plot a 7-DOF railway response with its saved model identity (requires matplotlib)."""
import argparse
import csv
import math
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("csv_file", type=Path)
args = parser.parse_args()
with args.csv_file.open(newline="", encoding="utf-8") as stream:
    rows = list(csv.DictReader(stream))
if len(rows) < 2:
    raise SystemExit("Need at least two data rows.")
columns = {key: [float(row[key]) for row in rows] for key in rows[0]}
if any(not math.isfinite(value) for values in columns.values() for value in values):
    raise SystemExit("CSV contains non-finite data.")
time = columns["time_s"]
if any(b <= a for a, b in zip(time, time[1:])):
    raise SystemExit("CSV time must increase strictly.")

fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True, constrained_layout=True)
metadata_path = args.csv_file.parent / "input_parameters.json"
metadata = json.loads(metadata_path.read_text(encoding="utf-8")) if metadata_path.exists() else {}
label = metadata.get("model_label", "Legacy illustrative train")
input_label = "File input: mean(Lv,Rv); Ld/Rd export-only" if metadata.get("track_file") else "Synthetic track input"
fig.suptitle(f"{label}: 7 vertical DOFs\n{input_label}; assumed support, unvalidated response")
for i in range(1, 5):
    axes[0].plot(time, [v * 1000 for v in columns[f"track_{i}_z_m"]], label=f"Axle {i}", linewidth=1)
axes[0].set_ylabel("Track input [mm]")
for name, label in [("car", "Carbody"), ("bogie_front", "Front bogie"), ("wheelset_1", "Leading wheelset")]:
    axes[1].plot(time, [v * 1000 for v in columns[f"{name}_dz_m"]], label=label, linewidth=1)
axes[1].set_ylabel("Displacement [mm]")
axes[2].plot(time, columns["car_az_m_s2"], label="Carbody (unweighted)", color="tab:green")
axes[2].set_ylabel("Acceleration [m/s²]")
for i in range(1, 5):
    axes[3].plot(time, [v / 1000 for v in columns[f"support_{i}_N"]], label=f"Axle {i}", linewidth=1)
axes[3].set_ylabel("Axle support [kN]")
axes[3].set_xlabel("Time [s]")
for axis in axes:
    axis.grid(alpha=0.25)
    axis.legend(loc="upper right", ncol=4, fontsize=8)
for suffix in (".png", ".svg"):
    target = args.csv_file.with_suffix(suffix)
    fig.savefig(target, dpi=160)
    print(target)
plt.close(fig)
