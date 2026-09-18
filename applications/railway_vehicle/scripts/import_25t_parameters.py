"""Import the supplied Puzhen 25T DOCX tables and create traceable vertical-model configurations."""
import argparse
import hashlib
import json
import zipfile
from pathlib import Path
from xml.etree import ElementTree as ET

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("document", type=Path)
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
ns = {"w": "http://schemas.openxmlformats.org/wordprocessingml/2006/main"}
with zipfile.ZipFile(args.document) as archive:
    body = ET.fromstring(archive.read("word/document.xml")).find("w:body", ns)
tables = []
blocks = []
for element in body:
    if element.tag.endswith("}tbl"):
        rows = [["".join(t.text or "" for t in cell.findall(".//w:t", ns))
                 for cell in row.findall("w:tc", ns)] for row in element.findall("w:tr", ns)]
        tables.append(rows)
        blocks.append("\n".join(" | ".join(row) for row in rows))
    else:
        blocks.append("".join(t.text or "" for t in element.findall(".//w:t", ns)))
source = {"path": str(args.document.resolve()), "sha256": hashlib.sha256(args.document.read_bytes()).hexdigest(),
          "tables": tables}
source_dir = root / "sources"
source_dir.mkdir(exist_ok=True)
(source_dir / "25t_document_tables.json").write_text(json.dumps(source, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
(source_dir / "25t_document_text.txt").write_text("\n\n".join(blocks) + "\n", encoding="utf-8")

def value(table, label):
    rows = [row for row in tables[table] if row and row[0] == label]
    if len(rows) != 1:
        raise ValueError(f"Cannot uniquely identify source row: {label}")
    return float(rows[0][1])

def characteristic(table):
    first, second = tables[table]
    assert first[0].startswith("V(") and second[0].startswith("F(")
    return [[0, 0]] + [[float(v), float(f)] for v, f in zip(first[1:], second[1:])]

# Index references are checked against labels so document layout changes fail explicitly.
assert len(tables) == 11, "Unexpected table count"
assert tables[10][0][1:6] == ["YZ", "YW", "RW", "CA", "控制车"]
base = {
    "source_document": {"path": source["path"], "sha256": source["sha256"]},
    "parameter_status": "Document-based vertical vehicle parameters; assumed track/support/speed; not experimentally validated",
    "bogie_mass_kg": value(0, "构架质量mf(簧间质量)") * 1000,
    "wheelset_mass_kg": value(0, "簧下质量（轮对质量＋轴箱质量）每轮对") * 1000,
    "bogie_spacing_m": value(0, "车辆定距2L"),
    "wheelbase_m": value(0, "转向架轴距2b") / 1000,
    "bogie_height_m": value(0, "构架重心距轨面高Hf") / 1000,
    "wheel_radius_m": value(0, "车轮滚动圆直径") / 2000,
    "primary_k_N_m": value(1, "一系钢簧垂向刚度(每轴箱)Kpz") * 1e6 * 2,
    "secondary_k_N_m": value(2, "二系垂向刚度(每空气簧)Ksy") * 1e6 * 2,
    "primary_lower_z_m": value(1, "一系簧下作用点距轨面高h2") / 1000,
    "primary_upper_z_m": value(1, "一系簧上作用点距轨面高h1") / 1000,
    "secondary_lower_z_m": value(2, "空气簧下表面距轨面高h3") / 1000,
    "secondary_upper_z_m": value(2, "空气簧上表面距轨面高h4") / 1000,
    "primary_dampers_per_axle": 2,
    "secondary_dampers_per_bogie": 2,
    "primary_damper_curve_m_s_N": characteristic(7),
    "secondary_damper_curve_m_s_N": characteristic(6),
    "support_k_N_m": 1e8, "support_c_Ns_m": 20000,
    "speed_m_s": 120 / 3.6,
    "track_amplitude_m": 0.001, "track_wavelength_m": 25,
    "track_start_m": 40, "track_length_m": 200, "gravity_m_s2": 9.81,
    "assumptions": {
        "speed": "120 km/h scenario assumption; no operating speed specified in source document",
        "support": "Legacy linear bilateral equivalent axle support, not Hertz contact; source does not give these values",
        "track": "Synthetic windowed sinusoid retained for comparison; no measured irregularity in document",
        "dampers": "Two per axle / two per bogie; odd symmetry, origin added, linear interpolation and last-segment extrapolation",
        "damper_joints": "Source joint stiffnesses recorded but not modeled; dampers act directly on relative vertical velocity",
        "reduction": "Seven vertical translations only; axlebox mass lumped, no pitch/roll/yaw or arm inertia",
        "air_spring": "Normal inflated linear equivalent; deflated case not modeled"
    },
    "reference_only_not_used": {
        "bare_wheelset_mass_kg": value(0, "轮对质量（带制动盘）mw") * 1000,
        "wheelset_inertia_kg_m2": [value(0, key) for key in ["轮对侧滚转动惯量（带制动盘）Iwx", "轮对点头转动惯量（带制动盘）Iwy", "轮对摇头转动惯量（带制动盘）Iwz"]],
        "bogie_inertia_kg_m2": [1000 * value(0, key) for key in ["构架侧滚转动惯量Ifx", "构架点头转动惯量Ify", "构架摇头转动惯量Ifz"]],
        "inertia_note": "Source says carbody parameter origin is at car center on rail plane. Do not assign directly as COM inertias without clarifying reference axes.",
        "primary_damper_joint_stiffness_N_m": value(3, "一系垂向减振器接头刚度") * 1e6,
        "secondary_damper_joint_stiffness_N_m": value(3, "二系垂向减振器接头刚度") * 1e6
    }
}
assert base["primary_damper_curve_m_s_N"][1:] == [[0.3,2600],[0.5,4095],[1.5,9650]]
assert base["secondary_damper_curve_m_s_N"][1:] == [[0.15,2250],[0.5,3115]]
for index, kind in enumerate(["YZ", "YW", "RW", "CA", "CONTROL"]):
    for load_index, load in enumerate(["empty", "loaded"]):
        config = json.loads(json.dumps(base))
        # Carbody rows have an extra load-condition column before the five variants.
        config["model_label"] = f"Puzhen 25T {kind} {load}"
        config["car_mass_kg"] = float(tables[10][1 + load_index][index + 2]) * 1000
        config["car_height_m"] = float(tables[10][9 + load_index][index + 2]) / 1000
        config["reference_only_not_used"]["car_inertia_source_kg_m2"] = [
            float(tables[10][row + load_index][index + 2]) * 1000 for row in [3, 5, 7]]
        path = root / "config" / f"25t_{kind.lower()}_{load}.json"
        path.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(path.name, config["car_mass_kg"], config["car_height_m"])
