"""
gain_schedule_generate_cases.py

Purpose
-------
Automates the 3-trim-point JSBSim workflow for the C172 gain-scheduling task:

  1. Takes your existing cruise run-script (c172_trim_linearize_final.xml)
     and cruise IC file (c172p_cruise_init.xml) as the template.
  2. Generates two additional IC files (low-speed, high-speed) by editing
     ONE trim-speed property in a copy of the cruise IC file.
  3. Generates two additional run-scripts that point at those new IC files.
  4. Runs JSBSim.exe once per case (low / cruise / high).
  5. Finds the linearization output (.sce file) produced by each run and
     saves it under a case-specific name, so the three don't overwrite
     each other.

WHAT YOU MUST DO BEFORE THE FIRST REAL RUN
-------------------------------------------
Run this script once with DRY_RUN_INSPECT_ONLY = True (default below).
It will NOT touch JSBSim. It will just print every leaf tag/value found
in your cruise IC file. Find the tag that holds trim airspeed (something
like vc-kts, vc-fps, ubody-fps, vt-fps -- naming varies by how the IC file
was authored) and put it in CONFIG['ic_speed_tag'] below, along with
whether it's expressed in knots or ft/s. Then set DRY_RUN_INSPECT_ONLY
to False and run again.

This script does not touch your original c172_trim_linearize_final.xml
or c172p_cruise_init.xml -- it only reads them and writes new copies.
"""

import copy
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

# ======================================================================
# CONFIG -- edit this block for your machine
# ======================================================================

CONFIG = {
    # Root of your JSBSim project (the "cd /d" target)
    "ctms_dir": r"D:\Controls\CTMS",

    # JSBSim executable
    "jsbsim_exe": r"D:\jsbsim-1.3.1\out\install\x64-Release\bin\JSBSim.exe",

    # Existing files, relative to ctms_dir, that you already validated
    "base_runscript": r"scripts\c172_trim_linearize_final.xml",
    "base_ic_file": r"aircraft\c172p\c172p_cruise_init.xml",  # <-- FIX THIS PATH if wrong

    # ------------------------------------------------------------------
    # FILL THIS IN AFTER RUNNING THE INSPECT STEP (see docstring above)
    # ------------------------------------------------------------------
    "ic_speed_tag": "vt",        # e.g. "vc-kts", "vc-fps", "ubody-fps"
    "ic_speed_unit": "fps",       # "kts" or "fps" -- must match the tag above

    # Trim cases: (case_name, target_airspeed_ft_per_sec, is_existing_cruise_case)
    # Cruise reuses your already-validated 174.426 ft/s case as-is.
    # NOTE: cruise = 184.999 ft/s TRUE airspeed (matches velocity_trim1/2 baked
    # into the .slx and U_trim in the design doc). Do NOT use 174.426 here --
    # that number is the CALIBRATED airspeed (vc-fps) JSBSim reports when
    # trimmed at true 184.999 ft/s and 4000 ft altitude. It is a different
    # physical quantity from vt/u_trim and must never be fed back into <vt>.
    "trim_cases": [
        ("low", 100.0, False),
        ("cruise", 184.999, True),
        ("high", 220.0, False),
    ],

    # Output folder (created under ctms_dir) where the three .sce files
    # and run logs land.
    "output_subdir": "gain_schedule_outputs",
}

# Set this False only after you've filled in ic_speed_tag / ic_speed_unit above.
DRY_RUN_INSPECT_ONLY = True

# ======================================================================
# Implementation -- shouldn't need edits below this line
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
        if len(elem) == 0:  # leaf node
            attrs = f" attrs={elem.attrib}" if elem.attrib else ""
            print(f"  <{elem.tag}>{elem.text!r}{attrs}")
    print("=== end dump ===\n")
    print("Find the tag that represents trim airspeed above, then set")
    print("CONFIG['ic_speed_tag'] and CONFIG['ic_speed_unit'] at the top")
    print("of this script, set DRY_RUN_INSPECT_ONLY = False, and rerun.\n")


def make_ic_file(base_ic_path: Path, out_ic_path: Path, tag: str, unit: str, target_ft_s: float) -> None:
    tree = ET.parse(base_ic_path)
    root = tree.getroot()

    matches = [e for e in root.iter() if e.tag == tag]
    if len(matches) == 0:
        raise RuntimeError(
            f"Tag <{tag}> not found in {base_ic_path}. "
            f"Re-run with DRY_RUN_INSPECT_ONLY=True and re-check CONFIG['ic_speed_tag']."
        )
    if len(matches) > 1:
        raise RuntimeError(
            f"Tag <{tag}> appears {len(matches)} times in {base_ic_path} -- ambiguous. "
            f"Pick a more specific tag or edit this script to disambiguate by parent."
        )

    new_value = ft_s_to_configured_unit(target_ft_s, unit)
    old_text = matches[0].text
    matches[0].text = f"{new_value:.6f}"
    print(f"  IC edit: <{tag}> {old_text} -> {matches[0].text} ({unit})")

    tree.write(out_ic_path, xml_declaration=True, encoding="UTF-8")


def make_runscript(base_script_path: Path, out_script_path: Path, new_ic_name: str) -> None:
    """
    Text-based edit (not ElementTree) so comments/formatting in the
    original run-script are preserved exactly except for the one
    attribute we change.
    """
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
        sys.exit(f"ERROR: base IC file not found: {base_ic_file}\n"
                  f"Fix CONFIG['base_ic_file'] to the real path first.")

    if DRY_RUN_INSPECT_ONLY:
        dump_ic_file_leaves(base_ic_file)
        return

    if not CONFIG["ic_speed_tag"] or not CONFIG["ic_speed_unit"]:
        sys.exit("ERROR: CONFIG['ic_speed_tag'] / CONFIG['ic_speed_unit'] not set. "
                 "Run with DRY_RUN_INSPECT_ONLY=True first.")

    out_dir = ctms_dir / CONFIG["output_subdir"]
    out_dir.mkdir(exist_ok=True)

    scripts_dir = base_runscript.parent
    ic_dir = base_ic_file.parent

    for case_name, target_v_ft_s, is_existing_cruise in CONFIG["trim_cases"]:
        print(f"\n--- Case: {case_name} (target {target_v_ft_s} ft/s) ---")

        if is_existing_cruise:
            run_script_path = base_runscript
            relative_script_path = CONFIG["base_runscript"].replace("\\", "/")
        else:
            new_ic_path = ic_dir / f"c172p_{case_name}_init.xml"
            make_ic_file(
                base_ic_file, new_ic_path,
                CONFIG["ic_speed_tag"], CONFIG["ic_speed_unit"], target_v_ft_s
            )
            new_ic_name_no_ext = new_ic_path.stem  # JSBSim refs IC files without extension

            new_script_path = scripts_dir / f"c172_trim_linearize_{case_name}.xml"
            make_runscript(base_runscript, new_script_path, new_ic_name_no_ext)

            run_script_path = new_script_path
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
            print(f"  WARNING: non-zero exit code for case '{case_name}'. "
                  f"Check {log_path} before trusting this case's .sce file.")

        sce_files = find_new_or_updated_sce([ctms_dir, scripts_dir], since_time)
        if not sce_files:
            print(f"  WARNING: no .sce file found updated after this run in "
                  f"{ctms_dir} or {scripts_dir}. Check where JSBSim writes it "
                  f"and adjust find_new_or_updated_sce()'s search_dirs if needed.")
            continue

        for sce in sce_files:
            dest = out_dir / f"c172_lin_{case_name}{sce.suffix}"
            shutil.copy2(sce, dest)
            print(f"  Saved: {dest}")

    print(f"\nDone. Check {out_dir} for the three c172_lin_<case>.sce files and logs.")
    print("Inspect each log's trim/linearization notify blocks before trusting the .sce output --")
    print("this script does not validate trim convergence for you.")


if __name__ == "__main__":
    main()