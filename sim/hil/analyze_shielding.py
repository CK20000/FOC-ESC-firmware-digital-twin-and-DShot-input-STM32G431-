import sys
import json
import math
import numpy as np
import matplotlib

matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Polygon
from matplotlib.collections import LineCollection
from matplotlib.path import Path

from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout

GEOMETRY_JSON = "pcb_geometry.json"
LAYERS = ["F_Cu", "In1_Cu", "In2_Cu", "In3_Cu", "In4_Cu", "B_Cu"]


def is_aggressor(net):
    if not net: return False
    n = net.upper()
    # Exclude logic and sense lines
    if any(x in n for x in (
    "UH", "UL", "VH", "VL", "WH", "WL", "TU", "TV", "TW", "SENSE", "GND", "3V3", "5V", "12V", "PA", "PB", "PC", "PD",
    "PE")):
        return False
    # Match actual power phases and battery
    return any(x in n for x in ("INVERTER", "PHASE", "BATT", "2U", "2V", "2W")) or n in ("U", "V", "W")


def is_victim(net):
    if not net: return False
    return "ADC" in net.upper()


def point_to_seg_dist(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    if dx == 0 and dy == 0: return math.hypot(px - x1, py - y1)
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


class ShieldingAnalyzer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ADC Broadside & Coplanar Analyzer")
        self.resize(1600, 900)

        with open(GEOMETRY_JSON) as f:
            self.geom = json.load(f)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        plt.style.use('dark_background')
        self.fig = Figure(figsize=(16, 10), facecolor='#111111')
        self.fig.subplots_adjust(left=0.05, right=0.95, top=0.92, bottom=0.05)
        self.canvas = FigureCanvas(self.fig)
        self.layout.addWidget(self.canvas)

        self.analyze()

    def analyze(self):
        tracks = self.geom.get("tracks", [])
        pours = self.geom.get("pours", [])
        footprints = self.geom.get("footprints", [])
        o = self.geom["outline"]
        bx, by, bw, bh = o["x_mm"], o["y_mm"], o["w_mm"], o["h_mm"]

        # 1. Filter Locations
        filter_coords = [(fp["x_mm"], fp["y_mm"], fp["ref"]) for fp in footprints if fp["ref"] in ("R16", "R19")]

        # 2. Extract Aggressors (Tracks + Pours)
        aggressor_items = []
        for t in tracks:
            if is_aggressor(t.get("net_name")):
                aggressor_items.append({"type": "track", "layer": t["layer"], "data": t, "net": t["net_name"]})
        for p in pours:
            if not p.get("is_hole") and is_aggressor(p.get("net_name")):
                pts = p["points"]
                # Fast bounding box for broadside checks
                xs = [pt[0] for pt in pts];
                ys = [pt[1] for pt in pts]
                bbox = (min(xs), min(ys), max(xs), max(ys))
                aggressor_items.append(
                    {"type": "pour", "layer": p["layer"], "data": p, "net": p["net_name"], "bbox": bbox})

        # 3. Extract Victims
        victim_tracks = [t for t in tracks if is_victim(t.get("net_name"))]

        print("=" * 70)
        print(f"[*] Found {len(aggressor_items)} Aggressor Items")
        print(f"[*] Found {len(victim_tracks)} Victim Items")
        print("=" * 70)

        results = []

        for v in victim_tracks:
            v_layer_idx = LAYERS.index(v["layer"])
            v_seg = ((v["x1"], v["y1"]), (v["x2"], v["y2"]))
            v_mid = ((v["x1"] + v["x2"]) / 2, (v["y1"] + v["y2"]) / 2)

            coplanar_dist = 999.0
            broadside_dist = 999.0
            worst_coplanar_net = ""
            worst_broadside_net = ""

            for a in aggressor_items:
                a_layer_idx = LAYERS.index(a["layer"])
                dist = 999.0

                # --- Distance Calculation ---
                if a["type"] == "track":
                    dist = point_to_seg_dist(v_mid[0], v_mid[1], a["data"]["x1"], a["data"]["y1"], a["data"]["x2"],
                                             a["data"]["y2"])
                else:
                    # Check bounding box first (fast)
                    bbox = a["bbox"]
                    if bbox[0] <= v_mid[0] <= bbox[2] and bbox[1] <= v_mid[1] <= bbox[3]:
                        dist = 0.0
                    else:
                        # Distance to bounding box edges
                        dx = max(bbox[0] - v_mid[0], 0, v_mid[0] - bbox[2])
                        dy = max(bbox[1] - v_mid[1], 0, v_mid[1] - bbox[3])
                        dist = math.hypot(dx, dy)

                # --- Coplanar (Same Layer) ---
                if a_layer_idx == v_layer_idx:
                    if dist < coplanar_dist:
                        coplanar_dist = dist
                        worst_coplanar_net = a["net"]
                # --- Broadside (Adjacent Layer) ---
                elif abs(a_layer_idx - v_layer_idx) == 1:
                    if dist < broadside_dist:
                        broadside_dist = dist
                        worst_broadside_net = a["net"]

            # --- Risk Scoring ---
            risk = "SAFE"
            color = "#00f0ff"  # Cyan

            if coplanar_dist < 1.0:
                risk = "HIGH (Coplanar)"
                color = "#ff0055"
            elif broadside_dist < 1.0:
                risk = "HIGH (Broadside)"
                color = "#ff5500"
            elif coplanar_dist < 2.0:
                risk = "MEDIUM (Coplanar)"
                color = "#ffcc00"
            elif broadside_dist < 2.0:
                risk = "MEDIUM (Broadside)"
                color = "#ffcc00"

            results.append({
                "track": v,
                "coplanar_dist": coplanar_dist,
                "broadside_dist": broadside_dist,
                "risk": risk,
                "color": color,
                "worst_coplanar_net": worst_coplanar_net,
                "worst_broadside_net": worst_broadside_net
            })

        # 4. Plotting
        ax = self.fig.add_subplot(111)
        ax.set_facecolor('#111111')
        ax.set_aspect('equal')
        ax.axis('off')
        ax.set_xlim(bx - 5, bx + bw + 5)
        ax.set_ylim(by + bh + 5, by - 5)
        ax.add_patch(plt.Rectangle((bx, by), bw, bh, fill=False, edgecolor='#333333', linewidth=0.5, linestyle='--'))

        # Draw Aggressors
        for a in aggressor_items:
            if a["type"] == "track":
                ax.plot([a["data"]["x1"], a["data"]["x2"]], [a["data"]["y1"], a["data"]["y2"]],
                        color='#ff0055', alpha=0.2, linewidth=1.5)
            else:
                ax.add_patch(
                    Polygon(a["data"]["points"], closed=True, facecolor='#ff0055', alpha=0.1, edgecolor='none'))

        # Draw Victims
        for r in results:
            v = r["track"]
            ax.plot([v["x1"], v["x2"]], [v["y1"], v["y2"]],
                    color=r["color"], linewidth=3, solid_capstyle='round')

            if r["risk"] != "SAFE":
                print(f"  {v['net_name']} ({v['layer']}):")
                print(f"    -> Coplanar Dist : {r['coplanar_dist']:.2f} mm (Nearest: {r['worst_coplanar_net']})")
                print(f"    -> Broadside Dist: {r['broadside_dist']:.2f} mm (Nearest: {r['worst_broadside_net']})")
                print(f"    -> RISK: {r['risk']}")

        # Draw Filter Markers
        for fx, fy, ref in filter_coords:
            ax.scatter([fx], [fy], s=200, marker='*', color='#00ff00', edgecolors='white', zorder=10)
            ax.text(fx + 1, fy + 1, f"{ref} (RC Filter)", color='#00ff00', fontsize=10, fontweight='bold', zorder=11)

        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], color='#ff0055', lw=2, alpha=0.5, label='Aggressor (Phase)'),
            Line2D([0], [0], color='#00f0ff', lw=3, label='ADC Trace: SAFE'),
            Line2D([0], [0], color='#ffcc00', lw=3, label='ADC Trace: MEDIUM RISK'),
            Line2D([0], [0], color='#ff0055', lw=3, label='ADC Trace: HIGH RISK'),
            Line2D([0], [0], marker='*', color='w', markerfacecolor='#00ff00', markersize=15, label='RC Filter')
        ]
        ax.legend(handles=legend_elements, loc='upper right', facecolor='#222222', edgecolor='#555555')

        ax.set_title("ADC Trace Shielding (Coplanar + Broadside)", color='white', fontsize=16, fontweight='bold')
        self.canvas.draw()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = ShieldingAnalyzer()
    win.show()
    sys.exit(app.exec())