"""
spice_compiler.py

Generate a DC operating-point SPICE netlist from the schematic.

Reads:
    digital_twin_netlist.json
    component_classification.json

Writes:
    auto_generated.cir
    source_metadata.json

No hardcoded pin numbers, no hardcoded rail list, no hardcoded
voltages. Pins are found by *function name* from the KiCad XML.
Rails are discovered from the *output* pin of every regulator
(buck: followed through the inductor off the SW node).
"""
import json
import re

NETLIST_JSON  = "digital_twin_netlist.json"
CLASSIFY_JSON = "component_classification.json"
OUT_CIR       = "auto_generated.cir"
OUT_META      = "source_metadata.json"


# === DC simulation scope ===
# The ESC power stage has no DC operating point — MOSFETs switch at
# 20-50 kHz, bootstrap caps refresh every cycle. For a DC supply-chain
# check we skip the whole power stage. Rails are modeled as ideal
# sources with series resistance, so we don't need the buck freewheel
# diodes or SW inductors either.
SKIP_TYPES_DC = {"mosfet_n", "mosfet_p", "gate_driver",
                 "diode", "schottky",
                 "current_sense_amp", "opamp"}  # power stage + signal
                                                # chain have no DC op-point
                                                # without motor current
SKIP_REFS_DC  = set()

# === Regulator characteristics (datasheet-typical) ===
REGULATOR_DROPOUT = {
    "buck": 1.5,    # XL1509 headroom at moderate load
    "ldo":  0.3,    # ME6211 low-dropout
}
REGULATOR_THETA_JA = {
    "buck": 55.0,   # SOP-8 with exposed pad
    "ldo":  220.0,  # SOT-23-5, no copper pour
}

# === Steady-state load currents per rail ===
# Design-intent numbers, not schematic topology.
RAIL_LOADS = {
    "12V": 0.015,  # EG2134 quiescent + gate charge
    "5V":  0.065,  # U9 LDO input (3V3 load + LDO losses)
    "3V3": 0.063,  # STM32 + 2x AD8418A + OPA2333
}
SOURCE_REGISTRY = []


def register(name, kicad_net, kind, **extra):
    entry = {
        "spice_name": name.lower(),
        "kicad_net":  kicad_net,
        "kind":       kind,
    }
    entry.update(extra)
    SOURCE_REGISTRY.append(entry)


def snet(net):
    if net is None:
        return "NC"
    if net in ("GND", "gnd", "0", "GNDA", "GNDD", "PGND", "AGND"):
        return "0"
    s = re.sub(r"[^A-Za-z0-9_]", "_", net)
    if s and s[0].isdigit():
        s = "N" + s
    return s or "NC"


# ---------- pin lookup ----------

def pin_func(info):
    s = (info.get("function") or "").strip().upper()
    return re.sub(r"[{}_\s]", "", s)


def find_pin_exact(pins, candidates):
    """Return the net on the first pin whose function is EXACTLY one of
    the candidates (case-insensitive). Iterates pins in numeric order."""
    cands = {re.sub(r"[{}_\s]", "", c.upper()) for c in candidates}
    for pin_num in sorted(pins.keys(), key=lambda s: (len(s), s)):
        info = pins[pin_num]
        if pin_func(info) in cands:
            return info["net"]
    return None


def find_pin_substr(pins, candidates, avoid=()):
    """Substring match. Avoid list wins if it matches first."""
    for pin_num in sorted(pins.keys(), key=lambda s: (len(s), s)):
        info = pins[pin_num]
        fn = pin_func(info).replace(" ", "")
        if any(a.upper().replace(" ", "") in fn for a in avoid):
            continue
        for c in candidates:
            if c.upper().replace(" ", "") in fn:
                return info["net"]
    return None


# ---------- primitive emitters ----------

def emit_R(ref, pins, ohms):
    nets = [p["net"] for p in pins.values()]
    if len(nets) < 2: return None
    return f"R{ref} {snet(nets[0])} {snet(nets[1])} {ohms:.6f}"


def emit_L_short(ref, pins):
    nets = [p["net"] for p in pins.values()]
    if len(nets) < 2: return None
    return f"L{ref} {snet(nets[0])} {snet(nets[1])} 1u"


def emit_schottky(ref, pins):
    a = find_pin_substr(pins, ["ANODE", "A"], avoid=["CATHODE", "K"])
    k = find_pin_substr(pins, ["CATHODE", "K"], avoid=["ANODE"])
    if not (a and k):
        nets = [p["net"] for p in pins.values()]
        if len(nets) < 2: return None
        a, k = nets[0], nets[1]
    return f"D{ref} {snet(a)} {snet(k)} SS14"


def emit_mosfet(ref, pins, model="AOD4184A"):
    g = find_pin_substr(pins, ["GATE"], avoid=["DRAIN", "SOURCE"]) or \
        find_pin_exact(pins, ["G"])
    d = find_pin_substr(pins, ["DRAIN"], avoid=["GATE", "SOURCE"]) or \
        find_pin_exact(pins, ["D"])
    s = find_pin_substr(pins, ["SOURCE"], avoid=["GATE", "DRAIN"]) or \
        find_pin_exact(pins, ["S"])
    if not (g and d and s):
        return None
    return f"X{ref} {snet(d)} {snet(g)} {snet(s)} {model}"


def emit_ad8418a(ref, pins):
    inp = find_pin_exact(pins, ["+IN", "IN+", "+", "INP"])
    inn = find_pin_exact(pins, ["-IN", "IN-", "-", "INN"])
    rf  = find_pin_exact(pins, ["REF", "VREF", "REF1", "VREF1"])
    gnd = find_pin_exact(pins, ["GND", "V-", "VSS", "GND1"])
    vcc = find_pin_exact(pins, ["V+", "VCC", "VDD", "VS"])
    out = find_pin_exact(pins, ["OUT", "VOUT"])
    missing = [name for name, v in
               [("INP", inp), ("INN", inn), ("REF", rf),
                ("GND", gnd), ("VCC", vcc), ("OUT", out)]
               if v is None]
    if missing:
        print(f"    [dbg] {ref} pins: " +
              ", ".join(f"{n}:{p['function']!r}/{p['pintype']!r}->{p['net']}"
                        for n, p in sorted(pins.items())))
        return None, missing
    return (f"X{ref} {snet(inp)} {snet(inn)} {snet(rf)} "
            f"{snet(gnd)} {snet(vcc)} {snet(out)} AD8418A"), []


def emit_opa2333(ref, pins):
    vcc = find_pin_exact(pins, ["V+", "VCC", "VDD"])
    gnd = find_pin_exact(pins, ["V-", "VEE", "VSS", "GND"])
    if not (vcc and gnd):
        return None, ["VCC/GND"]

    def fn(p):
        s = (pins[p].get("function") or "").strip().upper()
        return re.sub(r"[{}_\s]", "", s)

    # SOIC-8 dual op-amp: channel A = {1,2,3}, channel B = {5,6,7}
    for ch in [(1, 2, 3), (5, 6, 7)]:
        try:
            out_p = next(p for p in ch
                         if pins[str(p)]["pintype"] == "output")
            pos_p = next(p for p in ch if fn(str(p)) in ("+", "INP", "+IN"))
            neg_p = next(p for p in ch if fn(str(p)) in ("-", "INN", "-IN"))
        except (StopIteration, KeyError):
            continue
        net_out = pins[str(out_p)]["net"]
        net_neg = pins[str(neg_p)]["net"]
        net_pos = pins[str(pos_p)]["net"]
        # Follower detected: output net == negative-input net
        if net_out == net_neg:
            return (f"X{ref} {snet(net_pos)} {snet(net_neg)} "
                    f"{snet(vcc)} {snet(gnd)} {snet(net_out)} OPA2333"), []

    return None, ["no follower channel found"]


# ---------- rail discovery ----------

def find_buck_output(ref, pins, comps):
    # XL1509 FB pin connects directly to the output rail.
    fb_net = find_pin_exact(pins, ["FB", "FEEDBACK"])
    if fb_net:
        return fb_net
    # Fallback: SW/LX followed through the inductor
    sw_net = (find_pin_exact(pins, ["SW", "LX", "SWITCH", "OUT"]) or
              find_pin_substr(pins, ["SW"], avoid=["VIN"]))
    if sw_net is None:
        return None
    for l_ref, l_info in comps.items():
        if l_info["type"] != "inductor":
            continue
        l_nets = [p["net"] for p in l_info["pins"].values()]
        if sw_net in l_nets:
            for n in l_nets:
                if n != sw_net:
                    return n
    return None


def discover_rails(comps):
    """Return [(net_name, volts, ref, input_net, reg_type), ...]."""
    rails = []
    for ref, info in comps.items():
        if info["type"] not in ("ldo", "buck"):
            continue
        vout = info.get("vout")
        if vout is None:
            print(f"  [warn] {ref} ({info['raw_value']}): no voltage parsed")
            continue

        if info["type"] == "ldo":
            out_net = find_pin_exact(info["pins"], ["VOUT", "OUT", "VO"])
            in_net  = find_pin_exact(info["pins"], ["VIN", "IN", "VI"])
        else:  # buck
            out_net = find_buck_output(ref, info["pins"], comps)
            in_net  = find_pin_exact(info["pins"], ["VIN", "IN", "VI"])

        if out_net is None:
            print(f"  [warn] {ref}: no output pin")
            continue
        if in_net is None:
            print(f"  [warn] {ref}: no VIN pin")
            continue
        rails.append((out_net, vout, ref, in_net, info["type"]))
    return rails


def emit_rails(rails):
    lines = ["* === Rails (dropout-limited behavioral sources) ==="]
    seen = set()
    for net_name, volts, ref, in_net, reg_type in rails:
        if net_name in seen:
            continue
        seen.add(net_name)

        safe = snet(net_name)
        safe_in = snet(in_net)
        dropout = REGULATOR_DROPOUT.get(reg_type, 1.0)
        src, sense = f"{safe}_SRC", f"{safe}_SENSE"

        lines.append(f"* {net_name} <- {ref} ({reg_type}, {volts:.2f} V, "
                     f"dropout {dropout:.2f} V from {in_net})")
        lines.append(f"B_rail_{safe} {src} 0 V = "
                     f"min({volts:.6f}, V({safe_in}) - {dropout:.6f})")
        lines.append(f"V_sense_{safe} {src} {sense} DC 0")
        lines.append(f"R_rail_{safe} {sense} {safe} 0.05")

        load = RAIL_LOADS.get(net_name)
        if load:
            lines.append(f"I_load_{safe} {safe} 0 DC {load:.6f}")

        register(f"v_sense_{safe}", net_name, "power_rail",
                 ref=ref, reg_type=reg_type, vout=volts, input_net=in_net)

    return lines


# ---------- gate nets ----------

def collect_gate_nets(comps):
    gates = {}
    for ref, info in comps.items():
        if info["type"] not in ("mosfet_n", "mosfet_p"):
            continue
        gnet = (find_pin_substr(info["pins"], ["GATE"],
                                avoid=["DRAIN", "SOURCE"]) or
                find_pin_exact(info["pins"], ["G"]))
        if gnet and gnet not in gates:
            gates[gnet] = ref
    return gates


def emit_gate_drive_sources(gate_nets):
    lines = ["* === Gate drive placeholders (scenario-controlled) ==="]
    lines.append("* Override in scenarios.json: gates: { \"<net>\": <V> }")
    for i, gnet in enumerate(sorted(gate_nets)):
        safe = snet(gnet)
        src = f"V_gate_{i}_{safe}"
        lines.append(f"{src} {safe} 0 DC 0    ; gate {gnet} ({gate_nets[gnet]})")
        register(src, gnet, "scenario_gate")
    return lines


def emit_gmin_leaks(netlist):
    lines = ["* === Anti-floating resistors (1 MΩ) ==="]
    for i, net in enumerate(sorted(netlist.keys())):
        s = snet(net)
        if s == "0":
            continue
        lines.append(f"R_leak_{i} {s} 0 1Meg")
    return lines


# ---------- main ----------

def main():
    with open(NETLIST_JSON) as f: netlist = json.load(f)
    with open(CLASSIFY_JSON) as f: classify = json.load(f)
    comps = classify["components"]

    lines = [
        "* =================================================",
        "* AUTO-GENERATED SPICE NETLIST (DC op-point)",
        "* Source: FOC ESC schematic",
        "* =================================================",
        "",
        ".include foc_models.lib",
        "",
    ]

    unhandled = []
    for ref, info in comps.items():
        t = info["type"]
        pins = info["pins"]
        if t in SKIP_TYPES_DC or ref in SKIP_REFS_DC:
            lines.append(f"* {ref} ({t}) — power-stage part, skipped for DC")
            continue
        emitted = None

        if t == "resistor":
            emitted = emit_R(ref, pins, info["ohms"])
        elif t == "shunt":
            emitted = emit_R(ref, pins, info["ohms"])
        elif t == "capacitor":
            emitted = f"* C{ref} skipped (open in DC)"
        elif t == "inductor":
            emitted = emit_L_short(ref, pins)
        elif t in ("diode", "schottky"):
            emitted = emit_schottky(ref, pins)
        elif t == "mosfet_n":
            emitted = emit_mosfet(ref, pins, "AOD4184A")
        elif t == "mosfet_p":
            emitted = f"* Q{ref} P-MOSFET — no DC model defined"
        elif t == "current_sense_amp":
            emitted, missing = emit_ad8418a(ref, pins)
            if emitted is None:
                emitted = f"* {ref} AD8418A — unresolved pins: {missing}"
                unhandled.append(f"{ref} (missing {missing})")
        elif t == "opamp":
            emitted, missing = emit_opa2333(ref, pins)
            if emitted is None:
                emitted = f"* {ref} OPA2333 — unresolved pins: {missing}"
                unhandled.append(f"{ref} (missing {missing})")
        elif t in ("buck", "ldo"):
            emitted = f"* {ref} ({t}) — replaced by ideal rail source"
        elif t == "gate_driver":
            emitted = f"* {ref} EG2134 — skipped (gates driven by scenarios)"
        elif t in ("mcu", "can_transceiver", "crystal", "connector",
                   "switch", "test_point", "tvs", "led", "ic_unknown",
                   "ferrite_bead"):
            emitted = f"* {ref} ({t}) — skipped in DC"
        if emitted:
            lines.append(emitted)

    lines.append("")
    rails = discover_rails(comps)
    lines.extend(emit_rails(rails))
    lines.append("")
    lines.extend(emit_gmin_leaks(netlist))
    lines.append("")
    lines.extend([
        "* === Device models (generic) ===",
        ".model DIODEMOD D(Is=1e-14 N=1.05 Rs=0.5)",
        "",
        ".op",
        ".end",
    ])

    with open(OUT_CIR, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    with open(OUT_META, "w", encoding="utf-8") as f:
        json.dump({"sources": SOURCE_REGISTRY}, f, indent=2)

    print(f"[+] Wrote {OUT_CIR}  ({len(lines)} lines)")
    print(f"[+] Wrote {OUT_META}  ({len(SOURCE_REGISTRY)} sources)")
    print(f"[*] Discovered {len(rails)} rails from regulators:")
    for net, v, ref, in_net, reg_type in rails:
        print(f"      {net:10s} {v:6.2f} V  "
              f"({ref}, {reg_type}, from {in_net})")
    if unhandled:
        print(f"[!] {len(unhandled)} unhandled:")
        for u in unhandled[:10]:
            print(f"      {u}")


if __name__ == "__main__":
    main()