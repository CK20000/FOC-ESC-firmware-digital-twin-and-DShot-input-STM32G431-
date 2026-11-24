import sys
import json
import math
import numpy as np
import matplotlib

matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Polygon, Circle
from matplotlib.collections import PatchCollection
from matplotlib.colors import LogNorm
from matplotlib.cm import ScalarMappable
from matplotlib.ticker import FormatStrFormatter

from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout

GEOMETRY_JSON = "pcb_geometry.json"
LAYERS = ["F_Cu", "In1_Cu", "In2_Cu", "In3_Cu", "In4_Cu", "B_Cu"]
COPPER_RESISTIVITY = 1.72e-8
COPPER_THICKNESS_M = 35e-6


def is_power_path(net):
    if not net: return False
    n = "".join(c for c in net.upper() if c.isalnum())
    if any(n.endswith(p) for p in ("U", "V", "W", "TU", "TV", "TW", "BATT", "PHASE")):
        return True
    if any(b in n for b in ("SENSE", "PWM", "GATE", "DRV", "BOOT", "3V3", "5V", "12V")):
        return False
    return False


def is_gnd(net):
    return "GND" in net.upper()


class PolygonViewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Merged Copper & Power Vias")
        self.resize(1600, 900)

        with open(GEOMETRY_JSON) as f:
            self.geom = json.load(f)

        o = self.geom["outline"]
        self.bx, self.by, self.bw, self.bh = o["x_mm"], o["y_mm"], o["w_mm"], o["h_mm"]

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        self.fig, self.axs = plt.subplots(2, 3, figsize=(16, 8), facecolor='#111111')
        self.fig.subplots_adjust(left=0.02, right=0.90, top=0.92, bottom=0.05, wspace=0.05, hspace=0.15)
        self.canvas = FigureCanvas(self.fig)
        self.layout.addWidget(self.canvas)

        self.draw_polygons_by_layer()

    def draw_polygons_by_layer(self):
        pours = self.geom.get("pours", [])
        vias = self.geom.get("vias", [])

        all_power_r = []
        for p in pours:
            net = p.get("net_name") or ""
            if not is_power_path(net): continue
            pts = p.get("points", [])
            if len(pts) < 3: continue
            xs = [pt[0] for pt in pts];
            ys = [pt[1] for pt in pts]
            L_m = max(max(xs) - min(xs), max(ys) - min(ys)) / 1000.0
            W_m = min(max(xs) - min(xs), max(ys) - min(ys)) / 1000.0
            if W_m > 0 and L_m > 0:
                r_mohms = (COPPER_RESISTIVITY * L_m / (W_m * COPPER_THICKNESS_M)) * 1000.0
                all_power_r.append(r_mohms)

        if all_power_r:
            min_r, max_r = min(all_power_r), max(all_power_r)
            norm = LogNorm(vmin=max(min_r, 1e-2), vmax=max_r)
        else:
            norm = None

        cmap = matplotlib.colormaps['turbo']
        gnd_color = '#666666'

        for i, layer_name in enumerate(LAYERS):
            row = i // 3
            col = i % 3
            ax = self.axs[row, col]

            ax.set_facecolor('#111111')
            ax.set_aspect('equal')
            ax.axis('off')
            ax.set_xlim(self.bx - 2, self.bx + self.bw + 2)
            ax.set_ylim(self.by + self.bh + 2, self.by - 2)
            ax.set_title(layer_name, color='white', fontsize=14, pad=8, weight='bold')

            ax.add_patch(plt.Rectangle((self.bx, self.by), self.bw, self.bh,
                                       fill=False, edgecolor='#888888', linewidth=0.5, linestyle='--'))

            layer_polys = [p for p in pours if p.get("layer") == layer_name]

            # --- Pass 1: Draw GND Polygons ---
            gnd_patches = []
            for p in layer_polys:
                net = p.get("net_name") or ""
                if is_gnd(net):
                    pts = p.get("points", [])
                    if len(pts) >= 3:
                        gnd_patches.append(Polygon(pts, closed=True))

            if gnd_patches:
                pc_gnd = PatchCollection(gnd_patches, facecolor=gnd_color, edgecolor=gnd_color, linewidth=0.0)
                ax.add_collection(pc_gnd)

            # --- Pass 2: Categorize and Draw Vias ---
            gnd_via_patches = []
            conducting_via_patches = []

            for v in vias:
                net = v.get("net_name") or ""
                radius = v["diameter_mm"] / 2.0
                patch = Circle((v["x_mm"], v["y_mm"]), radius=radius)

                if is_gnd(net):
                    gnd_via_patches.append(patch)
                elif is_power_path(net):
                    conducting_via_patches.append(patch)
                # We IGNORE blocking vias entirely so the planes look solid!

            # Draw GND vias with the exact same color as the GND plane
            if gnd_via_patches:
                pc_gnd_via = PatchCollection(gnd_via_patches, facecolor=gnd_color, edgecolor=gnd_color, linewidth=0.0)
                ax.add_collection(pc_gnd_via)

            # Draw conducting power vias as bright gold
            if conducting_via_patches:
                pc_cond = PatchCollection(conducting_via_patches, facecolor='#d4af37', edgecolor='#8a6e1c',
                                          linewidth=0.3)
                ax.add_collection(pc_cond)

            # --- Pass 3: Draw Power Paths ---
            for p in layer_polys:
                net = p.get("net_name") or "UNKNOWN"
                if not is_power_path(net): continue

                pts = p.get("points", [])
                if len(pts) < 3: continue

                if norm is not None:
                    xs = [pt[0] for pt in pts];
                    ys = [pt[1] for pt in pts]
                    L_m = max(max(xs) - min(xs), max(ys) - min(ys)) / 1000.0
                    W_m = min(max(xs) - min(xs), max(ys) - min(ys)) / 1000.0
                    if W_m > 0 and L_m > 0:
                        r_mohms = (COPPER_RESISTIVITY * L_m / (W_m * COPPER_THICKNESS_M)) * 1000.0
                        face_color = cmap(norm(r_mohms))
                    else:
                        face_color = '#888888'
                else:
                    face_color = '#888888'

                poly = Polygon(pts, closed=True, facecolor=face_color, edgecolor=face_color, linewidth=0.0)
                ax.add_patch(poly)

                # Add net text
                if len(pts) > 20:
                    xs = [pt[0] for pt in pts];
                    ys = [pt[1] for pt in pts]
                    cx, cy = np.mean(xs), np.mean(ys)
                    disp_name = net.split("/")[-1] if "/" in net else net
                    ax.text(cx, cy, disp_name, color='white', fontsize=7,
                            ha='center', va='center', weight='bold',
                            bbox=dict(facecolor='black', alpha=0.7, pad=1, edgecolor='none'))

        # --- Add Colorbar ---
        if norm is not None:
            sm = ScalarMappable(norm=norm, cmap=cmap)
            sm.set_array([])
            cbar = self.fig.colorbar(sm, ax=self.axs, fraction=0.02, pad=0.02)
            cbar.set_label("Power Net Resistance (mΩ)", color="white", fontsize=12)
            cbar.ax.yaxis.set_tick_params(color="white")

            ticks = np.linspace(min_r, max_r, 5)
            cbar.set_ticks(ticks)
            cbar.ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
            plt.setp(cbar.ax.get_yticklabels(), color="white")

        self.canvas.draw()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = PolygonViewer()
    win.show()
    sys.exit(app.exec())