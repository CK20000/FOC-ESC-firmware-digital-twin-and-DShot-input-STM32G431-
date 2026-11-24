import json
import os
import subprocess
import sys

BOARD_FILE = "foc_g431_rev1.kicad_pcb"
OUTPUT_JSON = "digital_twin_netlist.json"

# --- Bootstrap: re-exec with KiCad's Python if pcbnew is missing ---
try:
    import pcbnew
except ImportError:
    KICAD_PY_CANDIDATES = [
        r"D:\kicad\bin\python.exe",
        r"C:\Program Files\KiCad\bin\python.exe",
    ]
    pf = r"C:\Program Files\KiCad"
    if os.path.isdir(pf):
        for d in sorted(os.listdir(pf), reverse=True):
            p = os.path.join(pf, d, "bin", "python.exe")
            if os.path.isfile(p): KICAD_PY_CANDIDATES.append(p)

    kicad_py = next((c for c in KICAD_PY_CANDIDATES if os.path.isfile(c)), None)

    if kicad_py is None:
        print("[!] pcbnew not importable and no KiCad Python found.")
        print("    Tried:")
        for c in KICAD_PY_CANDIDATES:
            print(f"      {c}")
        sys.exit(1)

    print(f"[*] Re-exec with KiCad Python: {kicad_py}")
    sys.exit(subprocess.run([kicad_py, __file__] + sys.argv[1:]).returncode)


# --- Main extraction logic ---
def extract_kicad_netlist(board_path, output_json_path):
    print(f"Loading board: {board_path}")
    try:
        board = pcbnew.LoadBoard(board_path)
    except Exception as e:
        print(f"Failed to load board: {e}")
        sys.exit(1)

    net_topology = {}
    for footprint in board.GetFootprints():
        ref = footprint.GetReference()
        val = footprint.GetValue()
        for pad in footprint.Pads():
            pad_num = pad.GetPadName()
            net_name = pad.GetNetname()
            if not net_name or net_name == "":
                continue
            if net_name not in net_topology:
                net_topology[net_name] = {"nodes": []}
            net_topology[net_name]["nodes"].append({
                "component": ref,
                "value": val,
                "pin": pad_num
            })

    with open(output_json_path, 'w') as f:
        json.dump(net_topology, f, indent=4)
    print(f"Successfully exported {len(net_topology)} nets to {output_json_path}")


if __name__ == "__main__":
    extract_kicad_netlist(BOARD_FILE, OUTPUT_JSON)