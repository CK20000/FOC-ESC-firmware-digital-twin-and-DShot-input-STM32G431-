"""
dc_scenario_solver.py

Run DC scenarios against auto_generated.cir.

Scenario keys reference KiCad net names directly:
    overrides       : {"+BATT": 16.8, "12V": 12.0, ...}
    gates           : {"<gate_net>": 10.0, ...}
    loads           : {"3V3": 0.06, "+BATT": 0.02, ...}
    extra_elements  : ["I_phase_u /current sensing/U 0 DC 20.0"]
    probes          : ["+BATT", "12V", "5V", "3V3"]
    show_currents   : true
"""
import argparse
import json
import os
import re
import subprocess
import sys

CIR_FILE     = "auto_generated.cir"
META_FILE    = "source_metadata.json"
SCEN_FILE    = "scenarios.json"

NGSPICE_EXE = r"C:\Users\user\Downloads\Spice64\bin\ngspice_con.exe"

REGULATOR_THETA_JA = {
    "buck": 55.0,
    "ldo":  220.0,
}


def snet(net):
    if net in ("GND", "gnd", "0"): return "0"
    s = re.sub(r"[^A-Za-z0-9_]", "_", net)
    if s and s[0].isdigit(): s = "N" + s
    return s or "NC"


def print_thermal(meta, nodes, branches, ambient_c=25.0):
    """Pdiss + Tj for every discovered power rail."""
    rows = []
    for src in meta:
        if src.get("kind") != "power_rail":
            continue
        ref = src.get("ref")
        reg_type = src.get("reg_type")
        in_net = src.get("input_net")
        if not (ref and reg_type and in_net):
            continue

        s_out = snet(src["kicad_net"])
        s_in  = snet(in_net)
        v_out = nodes.get(s_out, nodes.get(s_out.lower()))
        v_in  = nodes.get(s_in,  nodes.get(s_in.lower()))
        if v_out is None or v_in is None:
            continue

        branch_name = src["spice_name"]
        i_out = abs(branches.get(branch_name, 0.0))

        if reg_type == "ldo":
            p_diss = (v_in - v_out) * i_out
        else:
            eta = 0.85
            i_q = 0.005
            p_diss = v_out * i_out * (1.0 / eta - 1.0) + v_in * i_q

        theta = REGULATOR_THETA_JA.get(reg_type, 100.0)
        tj = ambient_c + p_diss * theta
        rows.append((ref, reg_type, v_in, v_out, i_out, p_diss, tj))

    if not rows:
        return
    print("  Thermal (T_amb = 25 C):")
    print(f"    {'Ref':5s} {'Type':5s} {'Vin':>7s} {'Vout':>7s} "
          f"{'Iout':>10s} {'Pdiss':>10s} {'Tj':>7s}")
    for ref, typ, vin, vout, iout, p, tj in rows:
        print(f"    {ref:5s} {typ:5s} {vin:7.2f} {vout:7.2f} "
              f"{iout*1e3:8.2f}mA {p*1e3:8.1f}mW {tj:6.1f}C")


def build_netlist(base_text, scen, meta_sources):
    rail_by_net = {s["kicad_net"]: s["spice_name"]
                   for s in meta_sources if s["kind"] == "power_rail"}
    gate_by_net = {s["kicad_net"]: s["spice_name"]
                   for s in meta_sources if s["kind"] == "scenario_gate"}

    extra = ["", f"* --- scenario: {scen['name']} ---"]

    # --- Voltage overrides ---
    for net_name, volts in scen.get("overrides", {}).items():
        spice_name = rail_by_net.get(net_name)
        if spice_name is not None:
            pat = rf"({re.escape(spice_name)}\s+\S+\s+0\s+DC\s+)[\d.eE+\-]+"
            new_text = re.sub(pat, rf"\g<1>{volts:.6f}", base_text,
                              flags=re.IGNORECASE)
            if new_text == base_text:
                print(f"  [warn] override regex missed on {net_name} "
                      f"(spice name {spice_name})")
            base_text = new_text
        else:
            # Not a discovered rail — create a fresh V-source for it
            safe = snet(net_name)
            extra.append(f"V_scen_{safe} {safe} 0 DC {volts:.6f}")

    # --- Gate overrides (must already have a gate source) ---
    for net_name, volts in scen.get("gates", {}).items():
        spice_name = gate_by_net.get(net_name)
        if spice_name is None:
            print(f"  [warn] no gate source on net {net_name}")
            continue
        pat = rf"({re.escape(spice_name)}\s+\S+\s+0\s+DC\s+)[\d.eE+\-]+"
        base_text = re.sub(pat, rf"\g<1>{volts:.6f}", base_text,
                           flags=re.IGNORECASE)

    # --- Loads + extras ---
    for net_name, amps in scen.get("loads", {}).items():
        extra.append(f"I_load_{snet(net_name)} {snet(net_name)} 0 DC {amps:.6f}")
    extra.extend(scen.get("extra_elements", []))

    out = []
    for ln in base_text.splitlines():
        if ln.strip().lower().startswith(".end"):
            out.extend(extra)
            out.append("")
        out.append(ln)
    return "\n".join(out)


def run_ngspice(ng, text, work_dir):
    path = os.path.join(work_dir, "_scenario_tmp.cir")
    with open(path, "w", newline="\r\n", encoding="utf-8") as f:
        f.write(text)
    try:
        r = subprocess.run([ng, "-b", path], capture_output=True, text=True,
                           timeout=30, encoding="utf-8", errors="replace",
                           cwd=work_dir)
        return r.stdout, r.stderr
    finally:
        try: os.remove(path)
        except OSError: pass


def parse_node_voltages(out):
    v, in_t = {}, False
    for line in out.splitlines():
        s = line.strip()
        if "Node" in s and "Voltage" in s: in_t = True; continue
        if in_t:
            if not s or s.startswith("Source"): break
            p = s.split()
            if len(p) >= 2:
                try: v[p[0]] = float(p[1])
                except ValueError: pass
    return v


def parse_branch_currents(out):
    c, in_t = {}, False
    for line in out.splitlines():
        s = line.strip()
        if "Source" in s and "Current" in s: in_t = True; continue
        if in_t:
            if not s:
                if c: break
                continue
            p = s.split()
            if len(p) >= 2 and p[0].endswith("#branch"):
                try: c[p[0][:-7]] = float(p[1])
                except ValueError: pass
    return c


def parse_powers(out):
    powers, lines, i = {}, out.splitlines(), 0
    while i < len(lines):
        s = lines[i].strip()
        if s == "Resistor: Simple linear resistor":
            i += 1
            if i >= len(lines) or not lines[i].strip().startswith("device"):
                i += 1; continue
            devs = lines[i].strip().split()[1:]
            n = len(devs)
            while i < len(lines):
                s2 = lines[i].strip()
                if s2.startswith("p "):
                    toks = s2.split()
                    if len(toks) >= 1 + n:
                        for d, p in zip(devs, toks[1:1+n]):
                            try: powers[d.upper()] = abs(float(p))
                            except ValueError: pass
                    break
                i += 1
        i += 1
    return powers


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default=None)
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    work_dir = os.getcwd()

    try:
        with open(CIR_FILE) as f: base = f.read()
    except FileNotFoundError:
        print(f"[!] {CIR_FILE} not found — run spice_compiler.py first")
        sys.exit(1)

    with open(SCEN_FILE) as f: scen_data = json.load(f)
    try:
        with open(META_FILE) as f: meta = json.load(f).get("sources", [])
    except FileNotFoundError:
        meta = []

    scenarios = scen_data["scenarios"]
    if args.list:
        for s in scenarios:
            print(f"  {s['name']:32s} {s.get('description','')}")
        return
    if args.scenario:
        scenarios = [s for s in scenarios if s["name"] == args.scenario]
        if not scenarios:
            print(f"[!] No scenario {args.scenario}"); sys.exit(1)

    if not os.path.isfile(NGSPICE_EXE):
        print(f"[!] ngspice not found at {NGSPICE_EXE}")
        sys.exit(1)
    print(f"[*] ngspice: {NGSPICE_EXE}")
    print(f"[*] {len(scenarios)} scenario(s)\n")

    for scen in scenarios:
        print("=" * 76)
        print(f"  SCENARIO: {scen['name']}")
        if scen.get("description"):
            print(f"  {scen['description']}")
        print("=" * 76)

        netlist = build_netlist(base, scen, meta)
        stdout, stderr = run_ngspice(NGSPICE_EXE, netlist, work_dir)

        nodes = parse_node_voltages(stdout)
        branches = parse_branch_currents(stdout)
        powers = parse_powers(stdout)

        if not nodes:
            print("  [!] No node voltages. ngspice stdout:")
            for ln in stdout.splitlines()[:25]:
                print(f"      {ln}")
            if stderr.strip():
                print("  [!] stderr:")
                for ln in stderr.splitlines()[:15]:
                    print(f"      {ln}")
            continue

        print("  Probes:")
        for probe in scen.get("probes", []):
            v = nodes.get(snet(probe)) or nodes.get(snet(probe).lower())
            if v is None:
                print(f"    {probe:24s} <not found>")
            else:
                print(f"    {probe:24s} {v:+10.5f} V")

        if scen.get("show_currents"):
            print("  Branch currents:")
            for b, i in sorted(branches.items()):
                print(f"    {b:26s} {i:+.6e} A")

        top = sorted(powers.items(), key=lambda kv: -kv[1])[:5]
        if top:
            print("  Top power dissipation:")
            for ref, p in top:
                print(f"    {ref:12s} {p*1e3:9.3f} mW")

        print_thermal(meta, nodes, branches)
        print()

    print("[+] Done.")


if __name__ == "__main__":
    main()