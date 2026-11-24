"""
export_schematic_netlist.py

Schematic -> digital_twin_netlist.json

Shells out to kicad-cli to export the XML netlist, then parses it
into the JSON schema used by netlist_analyzer.py / spice_compiler.py.

Each node now carries pinfunction and pintype so the compiler can
look pins up by name, not by number.
"""
import json
import os
import subprocess
import sys
import xml.etree.ElementTree as ET


# === CONFIG ===
KICAD_CLI       = r"D:\kicad\bin\kicad-cli.exe"
SCHEMATIC_FILE  = "foc_g431_rev1.kicad_sch"
NETLIST_XML     = "schematic.net"
OUTPUT_JSON     = "digital_twin_netlist.json"


def xml_to_topology(xml_path):
    tree = ET.parse(xml_path)
    root = tree.getroot()

    comp_values = {}
    for comp in root.findall("./components/comp"):
        ref = comp.get("ref")
        val_el = comp.find("value")
        comp_values[ref] = val_el.text if (val_el is not None and val_el.text) else ""

    topology = {}
    for net in root.findall("./nets/net"):
        net_name = net.get("name")
        if not net_name:
            continue
        nodes = []
        for node in net.findall("node"):
            nodes.append({
                "component":   node.get("ref"),
                "value":       comp_values.get(node.get("ref"), ""),
                "pin":         node.get("pin", "?"),
                "pinfunction": (node.get("pinfunction") or "").strip(),
                "pintype":     (node.get("pintype") or "").strip(),
            })
        topology[net_name] = {"nodes": nodes}
    return topology


def main():
    if not os.path.isfile(KICAD_CLI):
        print(f"[!] kicad-cli not found at {KICAD_CLI}")
        sys.exit(1)

    if not os.path.isfile(SCHEMATIC_FILE):
        print(f"[!] Schematic not found: {SCHEMATIC_FILE}")
        sys.exit(1)

    cmd = [KICAD_CLI, "sch", "export", "netlist",
           "--format", "kicadxml",
           "-o", NETLIST_XML,
           SCHEMATIC_FILE]
    print(f"[*] {' '.join(cmd)}")
    r = subprocess.run(cmd, capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    if r.returncode != 0:
        print(f"[!] kicad-cli failed (rc={r.returncode})")
        print(r.stdout)
        print(r.stderr)
        sys.exit(1)

    print(f"[*] Parsing {NETLIST_XML}")
    topo = xml_to_topology(NETLIST_XML)

    with open(OUTPUT_JSON, "w", encoding="utf-8") as f:
        json.dump(topo, f, indent=2)

    n_nodes = sum(len(v["nodes"]) for v in topo.values())
    n_comps = len({n["component"] for v in topo.values() for n in v["nodes"]})
    print(f"[+] {len(topo)} nets, {n_nodes} nodes, {n_comps} components")
    print(f"[+] Wrote {OUTPUT_JSON}")


if __name__ == "__main__":
    main()