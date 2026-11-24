import subprocess
import tempfile
import os
import re
import json

NGSPICE_EXE = r"C:\Users\user\Downloads\Spice64\bin\ngspice_con.exe"
NETLIST_TEMPLATE = "auto_generated_esc.cir"
TARGET_CURRENT = 30.0  # Amps

def run_ngspice(netlist_str):
    with tempfile.NamedTemporaryFile(mode="w", suffix=".cir", delete=False, newline="\r\n", encoding="utf-8") as f:
        f.write(netlist_str)
        temp_path = f.name
    try:
        r = subprocess.run([NGSPICE_EXE, "-b", temp_path], capture_output=True, text=True, timeout=15, encoding="utf-8", errors="replace")
        return r.stdout
    finally:
        try: os.unlink(temp_path)
        except OSError: pass

def main():
    powers = {}

    # 1. Automatically find the shunts
    shunts = []
    try:
        with open("component_classification.json") as f:
            comps = json.load(f)["components"]
            for ref, info in comps.items():
                if info["type"] == "resistor":
                    raw_val = str(info.get("raw_value", "")).strip().upper()
                    ohms = info.get("ohms", 1)
                    if ref in ("R22", "R23") or raw_val == "R" or (ohms is not None and ohms <= 0.005):
                        shunts.append(ref)
    except Exception as e:
        print(f"[!] Could not read classification JSON: {e}")

    if not shunts:
        shunts = ["R22", "R23"]
    else:
        print(f"[*] Found shunts: {shunts}")

    # 2. Inject power for FETs and Shunts
    for i in range(1, 13):
        powers[f"Q{i}"] = 0.5

    # --- TEMPORARY DEBUG: Turn off shunts to see FETs ---
    for ref in shunts:
        powers[ref] = 1.8  # <--- Set to 0.0 to hide the 90C hotspot
    # ---------------------------------------------------

    # 3. Power tree
    powers["U3"] = 0.183  # 5V Buck
    powers["U5"] = 0.158  # 12V Buck
    powers["U9"] = 0.107  # 3.3V LDO

    print(f"\n[+] Power Dissipation at {TARGET_CURRENT}A:")
    for ref, p in sorted(powers.items(), key=lambda x: -x[1]):
        print(f"    {ref:12s} : {p * 1000:8.2f} mW")

    with open("power_dissipation.json", "w") as f:
        json.dump(powers, f, indent=2)
    print(f"\n[+] Saved power_dissipation.json")

if __name__ == "__main__":
    main()