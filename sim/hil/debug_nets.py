import json

with open("pcb_geometry.json") as f:
    geom = json.load(f)

print("=== ADC TRACES ===")
for t in geom.get("tracks", []):
    net = t.get("net_name") or ""
    if "ADC" in net.upper():
        length = ((t["x2"]-t["x1"])**2 + (t["y2"]-t["y1"])**2)**0.5
        print(f"Net: {net:15s} | Layer: {t['layer']} | Width: {t['width_mm']} mm | Length: {length:.2f} mm")

print("\n=== GATE DRIVE NETS (Sample) ===")
for t in geom.get("tracks", []):
    net = t.get("net_name") or ""
    if any(w in net.upper() for w in ("UH", "UL", "HO", "LO", "Q1-G", "Q2-G", "Q1-S", "Q2-S")):
        length = ((t["x2"]-t["x1"])**2 + (t["y2"]-t["y1"])**2)**0.5
        print(f"Net: {net:15s} | Layer: {t['layer']} | Width: {t['width_mm']} mm | Length: {length:.2f} mm")