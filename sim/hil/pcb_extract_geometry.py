import json
import os
import subprocess
import sys

BOARD_FILE = "foc_g431_rev1.kicad_pcb"
OUTPUT_JSON = "pcb_geometry.json"

try:
    import pcbnew
except ImportError:
    KICAD_PY_CANDIDATES = [r"D:\kicad\bin\python.exe", r"C:\Program Files\KiCad\bin\python.exe"]
    pf = r"C:\Program Files\KiCad"
    if os.path.isdir(pf):
        for d in sorted(os.listdir(pf), reverse=True):
            p = os.path.join(pf, d, "bin", "python.exe")
            if os.path.isfile(p): KICAD_PY_CANDIDATES.append(p)
    kicad_py = next((c for c in KICAD_PY_CANDIDATES if os.path.isfile(c)), None)
    if kicad_py is None: sys.exit(1)
    sys.exit(subprocess.run([kicad_py, __file__] + sys.argv[1:]).returncode)

COPPER_LAYER_NAMES = {
    pcbnew.F_Cu: "F_Cu", pcbnew.In1_Cu: "In1_Cu", pcbnew.In2_Cu: "In2_Cu",
    pcbnew.In3_Cu: "In3_Cu", pcbnew.In4_Cu: "In4_Cu", pcbnew.B_Cu: "B_Cu",
}


def nm_to_mm(nm): return float(nm) / 1e6


def layer_name(layer_id): return COPPER_LAYER_NAMES.get(layer_id)


def find_net_at_point(board, x_nm, y_nm, layer_id):
    """Find the net name of the copper item located at (x, y) on layer_id."""
    pt = pcbnew.VECTOR2I(int(x_nm), int(y_nm))

    # 1. Check zones (pours)
    for zone in board.Zones():
        if zone.IsOnLayer(layer_id):
            try:
                polys = zone.GetFilledPolysList(layer_id)
                if polys.Contains(pt):
                    return zone.GetNetname()
            except Exception:
                pass

    # 2. Check tracks
    for item in board.GetTracks():
        if item.GetLayer() == layer_id and item.HitTest(pt):
            return item.GetNetname()

    # 3. Check pads
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.IsOnLayer(layer_id) and pad.HitTest(pt):
                return pad.GetNetname()

    return "UNKNOWN"


def extract():
    print(f"[*] Loading {BOARD_FILE}")
    board = pcbnew.LoadBoard(BOARD_FILE)
    bbox = board.GetBoardEdgesBoundingBox()
    outline = {
        "x_mm": nm_to_mm(bbox.GetX()), "y_mm": nm_to_mm(bbox.GetY()),
        "w_mm": nm_to_mm(bbox.GetWidth()), "h_mm": nm_to_mm(bbox.GetHeight()),
        "layer_count": board.GetCopperLayerCount(),
    }

    tracks, vias = [], []
    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            pos = t.GetPosition()
            vias.append({"x_mm": nm_to_mm(pos.x), "y_mm": nm_to_mm(pos.y), "diameter_mm": nm_to_mm(t.GetFrontWidth()),
                         "drill_mm": nm_to_mm(t.GetDrillValue())})
            continue
        lname = layer_name(t.GetLayer())
        if lname is None: continue
        s, e = t.GetStart(), t.GetEnd()
        tracks.append({
            "layer": lname,
            "x1": nm_to_mm(s.x), "y1": nm_to_mm(s.y),
            "x2": nm_to_mm(e.x), "y2": nm_to_mm(e.y),
            "width_mm": nm_to_mm(t.GetWidth()),
            "net_name": t.GetNetname(),
        })

    pours = []
    pour_counts = {L: 0 for L in COPPER_LAYER_NAMES.values()}
    for zone in board.Zones():
        for lid in COPPER_LAYER_NAMES.keys():
            if not zone.IsOnLayer(lid): continue
            try:
                poly = zone.GetFilledPolysList(lid)
            except Exception:
                continue
            for i in range(poly.OutlineCount()):
                chain = poly.Outline(i)
                pts = [[nm_to_mm(chain.CPoint(k).x), nm_to_mm(chain.CPoint(k).y)] for k in range(chain.PointCount())]
                if len(pts) >= 3:
                    pours.append(
                        {"layer": layer_name(lid), "points": pts, "net_name": zone.GetNetname(), "is_hole": (i > 0)})
                    pour_counts[layer_name(lid)] += 1

    print(f"[*] Pours found per layer: {pour_counts}")

    # --- EXTRACT SOLDER MASK OPENINGS ---
    mask_openings = []

    def process_mask_poly(pts, mask_layer_name, cu_layer_id):
        if len(pts) < 3: return
        # Calculate centroid to find the net underneath
        cx = sum(pt[0] for pt in pts) / len(pts)
        cy = sum(pt[1] for pt in pts) / len(pts)

        # Convert mm back to nm for KiCad API
        net = find_net_at_point(board, cx * 1e6, cy * 1e6, cu_layer_id)

        mask_openings.append({
            "layer": mask_layer_name,
            "points": pts,
            "net_name": net
        })

    for drawing in board.GetDrawings():
        if drawing.GetLayer() == pcbnew.F_Mask or drawing.GetLayer() == pcbnew.B_Mask:
            mask_layer_name = "F_Mask" if drawing.GetLayer() == pcbnew.F_Mask else "B_Mask"
            cu_layer = pcbnew.F_Cu if drawing.GetLayer() == pcbnew.F_Mask else pcbnew.B_Cu
            try:
                shape = drawing.GetEffectiveShape()
                if shape:
                    poly = shape.Outline()
                    pts = [[nm_to_mm(poly.CPoint(k).x), nm_to_mm(poly.CPoint(k).y)] for k in range(poly.PointCount())]
                    process_mask_poly(pts, mask_layer_name, cu_layer)
            except Exception:
                pass

    for zone in board.Zones():
        if zone.IsOnLayer(pcbnew.F_Mask) or zone.IsOnLayer(pcbnew.B_Mask):
            mask_layer_name = "F_Mask" if zone.IsOnLayer(pcbnew.F_Mask) else "B_Mask"
            cu_layer = pcbnew.F_Cu if zone.IsOnLayer(pcbnew.F_Mask) else pcbnew.B_Cu
            try:
                poly = zone.GetFilledPolysList(pcbnew.F_Mask if mask_layer_name == "F_Mask" else pcbnew.B_Mask)
            except Exception:
                continue
            for i in range(poly.OutlineCount()):
                chain = poly.Outline(i)
                pts = [[nm_to_mm(chain.CPoint(k).x), nm_to_mm(chain.CPoint(k).y)] for k in range(chain.PointCount())]
                process_mask_poly(pts, mask_layer_name, cu_layer)

    print(f"[*] Found {len(mask_openings)} solder mask openings")

    footprints = []
    for fp in board.GetFootprints():
        pos = fp.GetPosition()
        entry = {
            "ref": fp.GetReference(), "value": fp.GetValue(),
            "x_mm": nm_to_mm(pos.x), "y_mm": nm_to_mm(pos.y),
            "layer": "F_Cu" if fp.GetLayer() == pcbnew.F_Cu else "B_Cu",
            "pads": [], "silk_shapes": []
        }
        for pad in fp.Pads():
            p = pad.GetPosition();
            sz = pad.GetSize()
            entry["pads"].append(
                {"x_mm": nm_to_mm(p.x), "y_mm": nm_to_mm(p.y), "w_mm": nm_to_mm(sz.x), "h_mm": nm_to_mm(sz.y),
                 "angle_deg": pad.GetOrientationDegrees()})
        footprints.append(entry)

    with open(OUTPUT_JSON, "w") as f:
        json.dump({"outline": outline, "tracks": tracks, "vias": vias, "pours": pours, "footprints": footprints,
                   "mask_openings": mask_openings}, f)
    print(f"[+] Saved {OUTPUT_JSON}")


if __name__ == "__main__": extract()