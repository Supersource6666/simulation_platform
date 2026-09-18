"""Validate and snapshot s/Lv/Rv/Ld/Rd input without rescaling; generate measured-track run configuration."""
import argparse
import csv
import hashlib
import json
import math
import shutil
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("input_file", type=Path)
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
with args.input_file.open(encoding="utf-8-sig") as stream:
    header = stream.readline().split()
    if header != ["s", "Lv", "Rv", "Ld", "Rd"]:
        raise ValueError("Expected columns: s Lv Rv Ld Rd")
    rows = []
    for line_number, line in enumerate(stream, 2):
        if not line.strip():
            continue
        row = [float(cell) for cell in line.split()]
        if len(row) != 5 or not all(math.isfinite(v) for v in row):
            raise ValueError(f"Invalid row {line_number}")
        if rows and row[0] <= rows[-1][0]:
            raise ValueError("s must increase strictly")
        rows.append(row)
if len(rows) < 2:
    raise ValueError("Need at least two data points")
target = root / "data/origin_excitation_data.txt"
target.parent.mkdir(exist_ok=True)
if args.input_file.resolve() != target.resolve():
    shutil.copyfile(args.input_file, target)
digest = hashlib.sha256(target.read_bytes()).hexdigest()
gaps = [b[0]-a[0] for a,b in zip(rows, rows[1:])]
manifest = {
    "source_path": str(args.input_file.resolve()), "sha256": digest, "rows":len(rows),
    "columns": header, "units": "All five columns in metres; already converted by measurement2simpack.py",
    "semantics": {"s":"distance", "Lv":"left vertical", "Rv":"right vertical", "Ld":"left lateral", "Rd":"right lateral"},
    "first_s_m":rows[0][0], "last_s_m":rows[-1][0], "min_spacing_m":min(gaps), "max_spacing_m":max(gaps),
    "min_max_m": {name:[min(row[i] for row in rows),max(row[i] for row in rows)] for i,name in enumerate(header[1:],1)}
}
(root / "sources/track_input_manifest.json").write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
config = json.loads((root / "config/25t_yz_loaded.json").read_text(encoding="utf-8"))
config["track_file"] = "../data/origin_excitation_data.txt"
config["track_lead_in_m"] = 5
config["track_start_m"] = 40
config["track_source"] = manifest
config["parameter_status"] = "Document-based 25T and file-based vertical input; assumed support/speed; 7-DOF reduction; lateral channels not applied"
config["assumptions"]["track"] = "Imported source in metres; C1 PCHIP interpolation; 5 m synthetic lead-in before first sample; no extrapolation beyond last sample"
config["assumptions"]["track_reduction"] = "Use (Lv+Rv)/2 for symmetric vertical support; retain Ld/Rd for export only, no lateral/roll dynamics"
# Sine-only quantities are deliberately absent from a measured-file configuration.
for key in ("track_amplitude_m", "track_wavelength_m", "track_length_m"):
    config.pop(key, None)
(root / "config/25t_yz_loaded_measured.json").write_text(json.dumps(config,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
print(json.dumps(manifest,ensure_ascii=False,indent=2))
