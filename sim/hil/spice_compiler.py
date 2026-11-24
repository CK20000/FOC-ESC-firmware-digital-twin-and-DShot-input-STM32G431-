import json
import re

NETLIST_JSON = "digital_twin_netlist.json"
CLASSIFY_JSON = "component_classification.json"
OUTPUT_CIR = "auto_generated_esc.cir"


def spice_net(net_name):
    if net_name in ("GND", "gnd", "0"): return "0"
    s = re.sub(r"[^A-Za-z0-9_]", "_", net_name)
    if s[0].isdigit(): s = "N" + s
    return s


def build_pin_map(netlist):
    pin_map = {}
    for net_name, net_data in netlist.items():
        for node in net_data.get("nodes", []):
            pin_map.setdefault(node["component"], {})[node["pin"]] = net_name
    return pin_map


def main():
    with open(NETLIST_JSON) as f:
        netlist = json.load(f)
    with open(CLASSIFY_JSON) as f:
        classify = json.load(f)

    pin_map = build_pin_map(netlist)
    comps = classify["components"]

    lines = ["* ESC FOC AUTO-GENERATED SPICE NETLIST", ""]

    # 1. Passives
    for ref, info in comps.items():
        pins = pin_map.get(ref, {})
        if info["type"] == "resistor" and len(pins) >= 2:
            n1, n2 = list(pins.values())[:2]
            lines.append(f"R{ref} {spice_net(n1)} {spice_net(n2)} {info['ohms']}")
        elif info["type"] == "capacitor" and len(pins) >= 2:
            n1, n2 = list(pins.values())[:2]
            lines.append(f"C{ref} {spice_net(n1)} {spice_net(n2)} {info['farads']}")
        elif info["type"] == "schottky" and len(pins) >= 2:
            n1, n2 = list(pins.values())[:2]
            lines.append(f"D{ref} {spice_net(n1)} {spice_net(n2)} SS14")

    # 2. FETs (Phase U, V, W)
    # Assuming Q1..Q12 based on the layout. We will define a standard NMOS model.
    lines.append("\n* === MOSFETs ===")
    for ref, info in comps.items():
        if info["type"] == "mosfet":
            pins = pin_map.get(ref, {})
            if len(pins) >= 3:
                # Standard TO-252: 1=Gate, 2=Drain, 3=Source
                g = spice_net(pins.get("1", "0"))
                d = spice_net(pins.get("2", "0"))
                s = spice_net(pins.get("3", "0"))
                lines.append(f"M{ref} {d} {g} {s} {s} AOD4184A")

    # 3. Power Tree (Bucks & LDO)
    lines.append("\n* === Power Rails (Simplified for .op) ===")
    lines.append("V_BATT +BATT 0 DC 25.2")
    # 12V Buck (XL1509-12)
    lines.append("V_12V 12V 0 DC 12.0")
    # 5V Buck (XL1509-5)
    lines.append("V_5V 5V 0 DC 5.0")
    # 3.3V LDO (ME6211)
    lines.append("V_3V3 3V3 0 DC 3.3")

    # Loads for the power tree so ngspice computes the current
    lines.append("R_LOAD_MCU 3V3 0 50")  # 50mA MCU load
    lines.append("R_LOAD_GATE 12V 0 100")  # 12mA gate driver load

    # 4. Gate Drives (To be modified by solver)
    # Default all gates to 0V
    lines.append("\n* === Gate Drives (Solver will override) ===")
    for ref, info in comps.items():
        if info["type"] == "mosfet":
            pins = pin_map.get(ref, {})
            g = spice_net(pins.get("1", "0"))
            lines.append(f"V_G_{ref} {g} 0 DC 0")

    # 5. Motor Load Injection
    # We inject a DC current source between Phase U and Phase V
    lines.append("\n* === Motor Load (Solver will override) ===")
    lines.append("I_MOTOR_U_V PHASE_U PHASE_V DC 0")

    lines.append("\n.model AOD4184A NMOS(Vto=2.0 Kp=50 Cgdmax=1n Cgdmin=10p Cgs=2n Cjo=1n Is=1e-15 Rb=3m)")
    lines.append(".op")
    lines.append(".end")

    with open(OUTPUT_CIR, "w") as f:
        f.write("\n".join(lines))
    print(f"[+] Wrote {OUTPUT_CIR}")


if __name__ == "__main__":
    main()