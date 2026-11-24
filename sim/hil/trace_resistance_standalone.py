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
from matplotlib.collections import LineCollection, PatchCollection
from matplotlib.colors import LogNorm
from matplotlib.cm import ScalarMappable

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout,
    QHBoxLayout, QSlider, QLabel
)
from PyQt6.QtCore import Qt

GEOMETRY_JSON = "pcb_geometry.json"
COPPER_RESISTIVITY = 1.72e-8
COPPER_THICKNESS_M = 35e-6
BOARD_THICKNESS_M = 1.6e-3
PLATING_THICKNESS_M = 25e-6
MAX_CURRENT = 41.55


def calc_polygon_area(pts):
    """Shoelace formula to find the area of a polygon in mm^2"""
    area = 0.0
    n = len(pts)
    for i in range(n):
        j = (i + 1) % n
        area += pts[i][0] * pts[j][1]
        area -= pts[j][0] * pts[i][1]
    return abs(area) / 2.0


class TraceApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Standalone Trace Resistance")
        self.resize(1400, 800)

        try:
            with open(GEOMETRY_JSON) as f:
                self.geom = json.load(f)
        except FileNotFoundError:
            print("[!] pcb_geometry.json not found.")
            sys.exit(1)

        o = self.geom["outline"]
        self.bx, self.by, self.bw, self.bh = o["x_mm"], o["y_mm"], o["w_mm"], o["h_mm"]

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        # Slider
        self.slider_layout = QHBoxLayout()
        self.slider_label = QLabel(f"Current: 0.0 A")
        self.slider_label.setStyleSheet("font-size: 24px; font-weight: bold; color: #ffb733;")
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, int(MAX_CURRENT * 10))
        self.slider.setValue(0)
        self.slider.valueChanged.connect(self.update_current)

        self.slider_layout.addWidget(self.slider_label)
        self.slider_layout.addWidget(self.slider)
        self.layout.addLayout(self.slider_layout)

        # Matplotlib Canvas
        self.fig = Figure(figsize=(12, 6), facecolor='#050505')
        self.canvas = FigureCanvas(self.fig)
        self.layout.addWidget(self.canvas)

        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor('#050505')
        self.ax.set_aspect('equal')
        self.ax.axis('off')

        self.ax.set_xlim(self.bx - 5, self.bx + self.bw + 5)
        self.ax.set_ylim(self.by + self.bh + 5, self.by - 5)

        self.draw_board()

        # Info Box
        self.info = self.fig.text(0.02, 0.95, "", color="white", fontsize=10,
                                  family="monospace", verticalalignment="top",
                                  bbox=dict(facecolor="#101010", alpha=0.9, pad=10, edgecolor="#666"))

        self.update_current()

    def draw_board(self):
        all_r = []
        cmap = matplotlib.colormaps['turbo']

        poly_patches = []
        poly_resistances = []

        # 1. Draw ALL large copper pours (ignoring names, using area)
        for p in self.geom.get("pours", []):
            if p.get("is_hole", False): continue

            pts = p.get("points", [])
            if len(pts) < 3: continue

            area_mm2 = calc_polygon_area(pts)
            if area_mm2 < 5.0: continue  # Skip tiny slivers

            xs = [pt[0] for pt in pts]
            ys = [pt[1] for pt in pts]
            L_m = max(max(xs) - min(xs), max(ys) - min(ys)) / 1000.0
            W_m = min(max(xs) - min(xs), max(ys) - min(ys)) / 1000.0

            if W_m <= 0 or L_m <= 0: continue

            r_ohms = COPPER_RESISTIVITY * L_m / (W_m * COPPER_THICKNESS_M)
            poly_patches.append(Polygon(pts, closed=True))
            poly_resistances.append(r_ohms)
            all_r.append(r_ohms)

        # 2. Draw Tracks
        track_segs = []
        track_resistances = []

        for t in self.geom.get("tracks", []):
            net = t.get("net_name") or ""
            w = t["width_mm"]
            if w < 0.5: continue  # Filter out tiny logic traces

            # Exclude sense lines
            if any(b in net.upper() for b in ("SENSE", "TU", "TV", "TW")): continue

            x1, y1, x2, y2 = t["x1"], t["y1"], t["x2"], t["y2"]
            length_m = math.hypot(x2 - x1, y2 - y1) / 1000.0
            width_m = w / 1000.0

            if width_m <= 0 or length_m <= 0: continue

            r_ohms = COPPER_RESISTIVITY * length_m / (width_m * COPPER_THICKNESS_M)
            track_segs.append([(x1, y1), (x2, y2)])
            track_resistances.append(r_ohms)
            all_r.append(r_ohms)

        # 3. Calculate Vias
        via_x, via_y, via_r = [], [], []
        for v in self.geom.get("vias", []):
            drill_m = v["drill_mm"] / 1000.0
            r_barrel_m = (drill_m / 2.0) - PLATING_THICKNESS_M
            area_m2 = 2 * math.pi * r_barrel_m * PLATING_THICKNESS_M

            if area_m2 <= 0: continue
            r_ohms = COPPER_RESISTIVITY * BOARD_THICKNESS_M / area_m2

            via_x.append(v["x_mm"])
            via_y.append(v["y_mm"])
            via_r.append(r_ohms)
            all_r.append(r_ohms)

        if not all_r:
            print("[!] No valid power items found.")
            return

        min_r, max_r = min(all_r), max(all_r)
        norm = LogNorm(vmin=max(min_r, 1e-5), vmax=max_r)

        # Draw Pours
        if poly_patches:
            pc = PatchCollection(poly_patches, cmap=cmap, norm=norm, alpha=0.8, edgecolor='face', linewidth=0.1,
                                 zorder=1)
            pc.set_array(np.array(poly_resistances))
            self.ax.add_collection(pc)

        # Draw Tracks
        if track_segs:
            lc = LineCollection(track_segs, cmap=cmap, norm=norm, linewidths=2.0, edgecolors='none', zorder=2)
            lc.set_array(np.array(track_resistances))
            self.ax.add_collection(lc)

        # Draw Vias on top
        if via_x:
            self.ax.scatter(via_x, via_y, s=8, c=via_r, cmap=cmap, norm=norm, edgecolors='white', linewidths=0.2,
                            zorder=3)

        sm = ScalarMappable(norm=norm, cmap=cmap)
        sm.set_array([])
        cbar = self.fig.colorbar(sm, ax=self.ax, fraction=0.03, pad=0.02)
        cbar.set_label("Resistance (Ω)", color="white")
        cbar.ax.yaxis.set_tick_params(color="white")
        plt.setp(cbar.ax.get_yticklabels(), color="white")

    def update_current(self):
        current = self.slider.value() / 10.0
        self.slider_label.setText(f"Current: {current:.1f} A")

        path_len_m = 0.020
        path_width_m = 0.005
        r_phase = COPPER_RESISTIVITY * path_len_m / (path_width_m * COPPER_THICKNESS_M)

        v_drop = current * r_phase * 1000
        p_loss = (current ** 2) * r_phase

        msg = f"Estimated Phase Path (@ {current:.1f} A):\n"
        msg += f"  R_path (est) : {r_phase * 1000:.3f} mΩ\n"
        msg += f"  V_drop       : {v_drop:.1f} mV\n"
        msg += f"  P_loss       : {p_loss:.2f} W\n"

        self.info.set_text(msg)
        self.canvas.draw_idle()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = TraceApp()
    win.show()
    sys.exit(app.exec())