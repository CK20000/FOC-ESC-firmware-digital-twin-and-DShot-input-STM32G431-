#!/usr/bin/env python3
"""
retire_standalones.py

Moves stand-alone scripts that are now subsumed by unified_esc_dashboard.py
into a deprecated/ folder. Non-destructive — refuses to overwrite, keeps a
manifest so you can roll back with --restore.
"""

import argparse
import json
import shutil
import sys
from pathlib import Path

# Files safe to retire. Everything else in the folder is left untouched.
TO_RETIRE = [
    "thermal_solver.py",
    "thermal_viewer.py",
    "thermal_gui_live.py",
    "transient_thermal_viewer.py",
    "pcb_thermal_esc.py",
]

# Files that MUST exist for the unified dashboard to run.
# If any are missing, we abort before moving anything — moving thermal_solver.py
# when the dashboard still imports it would break the app.
REQUIRED_FOR_UNIFIED = [
    "unified_esc_dashboard.py",
    "pcb_geometry.json",
    "digital_twin_netlist.json",
]

# Files that depend on ThermalSolver but are NOT in TO_RETIRE.
# If any of these remain in the root directory, they will break after the move
# (ImportError at runtime). Warn but don't block.
ORPHANED_IMPORTERS = [
    "foc_spice_solver.py",   # if it ever grows a thermal import
]


def find_root() -> Path:
    """Locate the simulation folder — either CWD or the script's own dir."""
    cwd = Path.cwd()
    if (cwd / "unified_esc_dashboard.py").is_file():
        return cwd
    script_dir = Path(__file__).resolve().parent
    if (script_dir / "unified_esc_dashboard.py").is_file():
        return script_dir
    print("[!] Cannot find unified_esc_dashboard.py in CWD or script directory.")
    print(f"    CWD       : {cwd}")
    print(f"    Script dir: {script_dir}")
    sys.exit(1)


def check_prerequisites(root: Path) -> bool:
    ok = True
    for name in REQUIRED_FOR_UNIFIED:
        p = root / name
        if not p.is_file():
            print(f"[!] Missing required file: {name}")
            ok = False
    for name in ORPHANED_IMPORTERS:
        p = root / name
        if p.is_file():
            print(f"[!] WARNING: {name} still imports thermal_solver — "
                  f"it will break after the move.")
            print(f"    Move it to deprecated/ manually, or update its import.")
    return ok


def do_retire(root: Path) -> int:
    dep = root / "deprecated"
    dep.mkdir(exist_ok=True)

    manifest_path = dep / "_manifest.json"
    manifest = []
    if manifest_path.is_file():
        try:
            manifest = json.loads(manifest_path.read_text())
        except json.JSONDecodeError:
            manifest = []

    moved, skipped, missing = 0, 0, 0
    for name in TO_RETIRE:
        src = root / name
        dst = dep / name
        if not src.is_file():
            print(f"  [skip] {name:32s} (not present)")
            missing += 1
            continue
        if dst.exists():
            print(f"  [skip] {name:32s} (already in deprecated/, not overwriting)")
            skipped += 1
            continue
        shutil.move(str(src), str(dst))
        manifest.append({"file": name, "from": str(src), "to": str(dst)})
        print(f"  [move] {name:32s} -> deprecated/{name}")
        moved += 1

    manifest_path.write_text(json.dumps(manifest, indent=2))

    print()
    print(f"[+] Moved {moved}, skipped {skipped}, not found {missing}.")
    print(f"[+] Manifest: deprecated/_manifest.json")
    print(f"[+] To roll back:  python retire_standalones.py --restore")
    return 0


def do_restore(root: Path) -> int:
    dep = root / "deprecated"
    manifest_path = dep / "_manifest.json"
    if not manifest_path.is_file():
        print("[!] No manifest found — nothing to restore.")
        return 1

    manifest = json.loads(manifest_path.read_text())
    restored = 0
    for entry in manifest:
        src = dep / entry["file"]
        dst = root / entry["file"]
        if not src.is_file():
            print(f"  [skip] {entry['file']:32s} (not in deprecated/)")
            continue
        if dst.exists():
            print(f"  [skip] {entry['file']:32s} (already exists in root)")
            continue
        shutil.move(str(src), str(dst))
        print(f"  [restore] {entry['file']:32s} <- deprecated/")
        restored += 1

    print()
    print(f"[+] Restored {restored} files.")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--restore", action="store_true",
                    help="move files back from deprecated/ to root")
    ap.add_argument("--dry-run", action="store_true",
                    help="show what would happen without moving anything")
    args = ap.parse_args()

    root = find_root()
    print(f"[*] Working directory: {root}")
    print()

    if args.restore:
        return do_restore(root)

    if not check_prerequisites(root):
        print()
        print("[!] Aborting — fix the issues above first.")
        return 1

    if args.dry_run:
        print("[dry-run] would move:")
        for name in TO_RETIRE:
            marker = "  " if (root / name).is_file() else " (missing)"
            print(f"  {name}{marker}")
        return 0

    return do_retire(root)


if __name__ == "__main__":
    sys.exit(main())