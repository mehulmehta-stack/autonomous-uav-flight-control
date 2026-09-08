"""
gain_schedule_generate_stress_cases.py

Extends the validated 3-point trim/linearization workflow to two
off-nominal (envelope-excursion) speeds:
  - 90 ft/s  : just above computed stall (~83.4 ft/s at 4000 ft, from
               the CLwbh table in c172p.xml) -- tests the LOW-speed
               extrapolation of the gain schedule.
  - 225 ft/s : just above the 220 ft/s design ceiling, still below
               Vno (~227.5 ft/s TAS at 4000 ft) -- tests the HIGH-speed
               extrapolation.

This does NOT touch your original c172_trim_linearize_final.xml or
c172p_cruise_init.xml, and it writes to a SEPARATE output folder so it
cannot overwrite your existing validated 100/185/220 .sce files.

ic_speed_tag/unit are already known-good from your original run
(tag="vt", unit="fps") -- no DRY_RUN inspection step needed.
"""

import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

# ======================================================================
# CONFIG
# ======================================================================

CONFIG = {
    "ctms_dir": r"D:\Controls\CTMS",
    "jsbsim_exe": r"D:\jsbsim-1.3.1\out\install\x64-Release\bin\JSBSim.exe",

    "base_runscript": r"scripts\c172_trim_linearize_final.xml",
    "base_ic_file": r"aircraft\c172p\c172p_cruise_init.xml",

    "ic_speed_tag": "vt",
    "ic_speed_unit": "fps",

    # Only the two NEW envelope-excursion cases. Your 100/185/220 points
    # already exist and are not re-run here.
    "trim_cases": [
        ("stress_low", 90.0, False),
        ("stress_high", 225.0, False),
    ],

    # Separate output folder -- will not touch your original
    # gain_schedule_outputs folder.
    "output_subdir": "gain_schedule_stress_outputs",
}

DRY_RUN_INSPECT_ONLY = True

# ======================================================================
# Implementation (unchanged from your working version)
# ======================================================================

FT_PER_SEC_PER_KT = 1.68781


def ft_s_to_configured_unit(v_ft_s: float, unit: str) -> float:
    if unit == "fps":
        return v_ft_s
    if unit == "kts":
        return v_ft_s / FT_PER_SEC_PER_KT
    raise ValueError(f"Unknown ic_speed_unit: {unit!r} (expected 'fps' or 'kts')")


def dump_ic_file_leaves(ic_path: Path) -> None:
    print(f"\n=== Leaf tags/values in {ic_path} ===")
    tree = ET.parse(ic_path)
    root = tree.getroot()
    for elem in root.iter():
        if len(elem) == 0:
            attrs = f" attrs={elem.attrib}" if elem.attrib else ""
            print(f"  <{elem.tag}>{elem.text!r}{attrs}")
    print("=== end dump ===\n")


def make_ic_file(base_ic_path: Path, out_ic_path: Path, tag: str, unit: str, target_ft_s: float) -> None:
    tree = ET.parse(base_ic_path)
    root = tree.getroot()

    matches = [e for e in root.iter() if e.tag == tag]
    if len(matches) == 0:
        raise RuntimeError(f"Tag <{tag}> not found in {base_ic_path}.")
    if len(matches) > 1:
        raise RuntimeError(f"Tag <{tag}> appears {len(matches)} times in {base_ic_path} -- ambiguous.")

    new_value = ft_s_to_configured_unit(target_ft_s, unit)
    old_text = matches[0].text
    matches[0].text = f"{new_value:.6f}"
    print(f"  IC edit: <{tag}> {old_text} -> {matches[0].text} ({unit})")

    tree.write(out_ic_path, xml_declaration=True, encoding="UTF-8")


def make_runscript(base_script_path: Path, out_script_path: Path, new_ic_name: str) -> None:
    text = base_script_path.read_text(encoding="utf-8")

    old_use_line_marker = 'initialize="'
    if old_use_line_marker not in text:
        raise RuntimeError(f"Could not find initialize=\"...\" in {base_script_path}")

    start = text.index(old_use_line_marker) + len(old_use_line_marker)
    end = text.index('"', start)
    old_ic_name = text[start:end]
    new_text = text[:start] + new_ic_name + text[end:]

    print(f"  Runscript edit: initialize=\"{old_ic_name}\" -> initialize=\"{new_ic_name}\"")
    out_script_path.write_text(new_text, encoding="utf-8")


def find_new_or_updated_sce(search_dirs, since_time: float):
    found = []
    for d in search_dirs:
        if not d.exists():
            continue
        for f in d.glob("*.sce"):
            if f.stat().st_mtime >= since_time:
                found.append(f)
    return found


def run_jsbsim(ctms_dir: Path, jsbsim_exe: Path, relative_script_path: str, log_path: Path) -> int:
    cmd = [str(jsbsim_exe), f"--script={relative_script_path}"]
    print(f"  Running: {' '.join(cmd)}  (cwd={ctms_dir})")
    with open(log_path, "w", encoding="utf-8") as logf:
        proc = subprocess.run(
            cmd, cwd=str(ctms_dir), stdout=logf, stderr=subprocess.STDOUT, text=True
        )
    return proc.returncode


def tail_file(path: Path, n_lines: int = 25) -> str:
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    return "\n".join(lines[-n_lines:])


def main():
    ctms_dir = Path(CONFIG["ctms_dir"])
    jsbsim_exe = Path(CONFIG["jsbsim_exe"])
    base_runscript = ctms_dir / CONFIG["base_runscript"]
    base_ic_file = ctms_dir / CONFIG["base_ic_file"]

    if not base_runscript.exists():
        sys.exit(f"ERROR: base runscript not found: {base_runscript}")
    if not base_ic_file.exists():
        sys.exit(f"ERROR: base IC file not found: {base_ic_file}")

    if DRY_RUN_INSPECT_ONLY:
        dump_ic_file_leaves(base_ic_file)
        return

    out_dir = ctms_dir / CONFIG["output_subdir"]
    out_dir.mkdir(exist_ok=True)

    scripts_dir = base_runscript.parent
    ic_dir = base_ic_file.parent

    for case_name, target_v_ft_s, is_existing_cruise in CONFIG["trim_cases"]:
        print(f"\n--- Case: {case_name} (target {target_v_ft_s} ft/s) ---")

        new_ic_path = ic_dir / f"c172p_{case_name}_init.xml"
        make_ic_file(base_ic_file, new_ic_path, CONFIG["ic_speed_tag"], CONFIG["ic_speed_unit"], target_v_ft_s)
        new_ic_name_no_ext = new_ic_path.stem

        new_script_path = scripts_dir / f"c172_trim_linearize_{case_name}.xml"
        make_runscript(base_runscript, new_script_path, new_ic_name_no_ext)

        relative_script_path = str(
            Path(CONFIG["base_runscript"]).parent / new_script_path.name
        ).replace("\\", "/")

        since_time = time.time()
        log_path = out_dir / f"jsbsim_log_{case_name}.txt"

        rc = run_jsbsim(ctms_dir, jsbsim_exe, relative_script_path, log_path)
        print(f"  JSBSim exit code: {rc}")
        print(f"  --- last lines of log ({log_path.name}) ---")
        print(tail_file(log_path))

        if rc != 0:
            print(f"  WARNING: non-zero exit code for '{case_name}'. This itself may be a "
                  f"real finding (trim failing to converge near stall) -- check {log_path} "
                  f"before assuming it's a bug to fix.")

        sce_files = find_new_or_updated_sce([ctms_dir, scripts_dir], since_time)
        if not sce_files:
            print(f"  WARNING: no .sce file found updated after this run.")
            continue

        for sce in sce_files:
            dest = out_dir / f"c172_lin_{case_name}{sce.suffix}"
            shutil.copy2(sce, dest)
            print(f"  Saved: {dest}")

    print(f"\nDone. Check {out_dir} for c172_lin_stress_low.sce and c172_lin_stress_high.sce.")


if __name__ == "__main__":
    main()