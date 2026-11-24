import json
import re

NETLIST_JSON  = "digital_twin_netlist.json"
OUTPUT_JSON   = "component_classification.json"

def parse_resistance(v):
    if v is None: return None
    s = str(v).strip().replace("R", "").replace("r", "").replace("Ω", "").replace("m", "e-3")
    m = re.match(r"^([0-9]*\.?[0-9]+)\s*([kKmM]?)$", s)
    if not m: return None
    num = float(m.group(1))
    mult = {"": 1, "k": 1e3, "K": 1e3, "m": 1e-3, "M": 1e6}[m.group(2)]
    return num * mult

def parse_capacitance(v):
    if v is None: return None
    s = str(v).strip().replace("F", "").replace("f", "")
    m = re.match(r"^([0-9]*\.?[0-9]+)\s*([pnuµm]?)$", s)
    if not m: return None
    num = float(m.group(1))
    mult = {"": 1.0, "p": 1e-12, "n": 1e-9, "u": 1e-6, "µ": 1e-6, "m": 1e-3}[m.group(2)]
    return num * mult

def classify(ref, value):
    ref = ref.upper()
    val = (value or "").upper()

    if ref.startswith("R"):
        ohms = parse_resistance(value)
        if ohms is not None:
            return {"type": "resistor", "ohms": ohms, "raw_value": value}

    if ref.startswith("C"):
        f = parse_capacitance(value)
        if f is not None:
            return {"type": "capacitor", "farads": f, "raw_value": value}

    if "AOD4184" in val:
        return {"type": "mosfet", "raw_value": value, "rds_on": 0.0067}
    if "ME6211" in val:
        return {"type": "ldo", "vout": 3.3, "raw_value": value}
    if "XL1509" in val:
        vout = 12.0 if "12" in val else 5.0
        return {"type": "buck", "vout": vout, "raw_value": value}
    if "EG2134" in val:
        return {"type": "gate_driver", "raw_value": value}
    if "AD8418" in val:
        return {"type": "opamp", "raw_value": value}
    if "SS14" in val:
        return {"type": "schottky", "raw_value": value}
    if "STM32G431" in val:
        return {"type": "mcu", "raw_value": value}

    return {"type": "unknown", "raw_value": value}

def analyze():
    with open(NETLIST_JSON) as f:
        netlist = json.load(f)
    components = {}
    for net_name, net_data in netlist.items():
        for node in net_data.get("nodes", []):
            ref = node["component"]
            val = node.get("value")
            if ref not in components:
                components[ref] = {"ref": ref, "value": val, "nets": []}
            if net_name not in components[ref]["nets"]:
                components[ref]["nets"].append(net_name)

    classified = {}
    for ref, entry in components.items():
        cls = classify(ref, entry["value"])
        cls["nets"] = entry["nets"]
        classified[ref] = cls

    with open(OUTPUT_JSON, "w") as f:
        json.dump({"components": classified}, f, indent=2)
    print(f"[+] Classified {len(classified)} components -> {OUTPUT_JSON}")

if __name__ == "__main__":
    analyze()