import sys
import json
import math
import numpy as np
import matplotlib

matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Polygon, Rectangle, Circle
from matplotlib.path import Path

from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout

GEOMETRY_JSON = "pcb_geometry.json"
LAYERS = ["F_Cu", "In1_Cu", "In2_Cu", "In3_Cu", "In4_Cu", "B_Cu"]
GRID_DX_MM = 0.5  # 0.5mm resolution for clear pad visualization
LOW_SIDE_FETS = ["Q2", "Q4", "Q6", "Q8", "Q10", "Q12"]


class TensorValidator(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Final Tensor Validation (Actual Pads)")
        self.resize(1600, 900)

        with open(GEOMETRY_JSON) as f:
            self.geom = json.load(f)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        plt.style.use('dark_background')
        self.fig = Figure(figsize=(16, 10), facecolor='#111111')
        self.fig.subplots_adjust(left=0.05, right=0.92, top=0.92, bottom=0.05)
        self.canvas = FigureCanvas(self.fig)
        self.layout.addWidget(self.canvas)

        self.build_and_visualize()

    def build_and_visualize(self):
        o = self.geom["outline"]
        bx, by, bw, bh = o["x_mm"], o["y_mm"], o["w_mm"], o["h_mm"]
        nx = int(math.ceil(bw / GRID_DX_MM))
        ny = int(math.ceil(bh / GRID_DX_MM))
        nz = 6

        print(f"[*] Building Tensor Grid: {nz} layers x {ny} rows x {nx} cols (Res: {GRID_DX_MM}mm)")

        tensor = [[[{"copper": 0.0, "vias": 0, "tags": []}
                    for x in range(nx)] for y in range(ny)] for z in range(nz)]

        # 1. Rasterize Copper Pours (IGNORE is_hole)
        for p in self.geom.get("pours", []):
            layer = p.get("layer")
            if layer not in LAYERS: continue
            if "GND" not in p.get("net_name", "").upper(): continue

            # Ignored the is_hole check here so we get the full copper plane
            z = LAYERS.index(layer)
            path = Path(p["points"])
            xs = [pt[0] for pt in p["points"]];
            ys = [pt[1] for pt in p["points"]]
            j0 = max(0, int((min(xs) - bx) / GRID_DX_MM));
            j1 = min(nx - 1, int((max(xs) - bx) / GRID_DX_MM))
            i0 = max(0, int((min(ys) - by) / GRID_DX_MM));
            i1 = min(ny - 1, int((max(ys) - by) / GRID_DX_MM))
            if j1 < j0 or i1 < i0: continue

            xx, yy = np.meshgrid(bx + (np.arange(j0, j1 + 1) + 0.5) * GRID_DX_MM,
                                 by + (np.arange(i0, i1 + 1) + 0.5) * GRID_DX_MM)
            test_pts = np.column_stack((xx.flatten(), yy.flatten()))
            inside = path.contains_points(test_pts).reshape((i1 - i0 + 1, j1 - j0 + 1))
            for i_local, i_global in enumerate(range(i0, i1 + 1)):
                for j_local, j_global in enumerate(range(j0, j1 + 1)):
                    if inside[i_local, j_local]:
                        tensor[z][i_global][j_global]["copper"] = 1.0

        # 2. Rasterize Copper Tracks
        for t in self.geom.get("tracks", []):
            layer = t.get("layer")
            if layer not in LAYERS: continue
            if "GND" not in t.get("net_name", "").upper(): continue

            z = LAYERS.index(layer)
            x1, y1, x2, y2 = t["x1"], t["y1"], t["x2"], t["y2"]
            length = math.hypot(x2 - x1, y2 - y1)
            steps = max(2, int(length / (GRID_DX_MM / 2)))

            for s in range(steps + 1):
                f = s / steps
                px = x1 + (x2 - x1) * f
                py = y1 + (y2 - y1) * f
                j = int((px - bx) / GRID_DX_MM)
                i = int((py - by) / GRID_DX_MM)
                if 0 <= i < ny and 0 <= j < nx:
                    tensor[z][i][j]["copper"] = 1.0

        # 3. Rasterize Vias
        for v in self.geom.get("vias", []):
            j = int((v["x_mm"] - bx) / GRID_DX_MM)
            i = int((v["y_mm"] - by) / GRID_DX_MM)
            if 0 <= i < ny and 0 <= j < nx:
                for z in range(nz):
                    tensor[z][i][j]["vias"] += 1

        # 5. Plotting
        for z in range(nz):
            ax = self.fig.add_subplot(2, 3, z + 1)
            ax.set_facecolor('#111111')
            ax.set_aspect('equal')
            ax.axis('off')
            ax.set_xlim(bx - 2, bx + bw + 2)
            ax.set_ylim(by + bh + 2, by - 2)
            ax.set_title(f"{LAYERS[z]}", color='white', fontweight='bold')

            # Draw Copper Grid
            copper_grid = np.zeros((ny, nx))
            for i in range(ny):
                for j in range(nx):
                    if tensor[z][i][j]["copper"] > 0.5:
                        copper_grid[i, j] = 1

            ax.imshow(copper_grid, extent=[bx, bx + nx * GRID_DX_MM, by + ny * GRID_DX_MM, by],
                      origin='upper', cmap='copper', alpha=0.6, interpolation='nearest')

            # Draw Vias (tiny white dots)
            for i in range(ny):
                for j in range(nx):
                    if tensor[z][i][j]["vias"] > 0:
                        ax.plot(bx + (j + 0.5) * GRID_DX_MM, by + (i + 0.5) * GRID_DX_MM, '.', color='white',
                                markersize=1)

            # Draw Actual Footprint Pads (Only for FETs and TP5)
            for fp in self.geom.get("footprints", []):
                ref = fp["ref"].upper()
                if ref != "TP5" and ref not in LOW_SIDE_FETS: continue

                # Only draw on the layer the component is placed on
                fp_layer = 0 if fp.get("layer") == "F_Cu" else 5
                if fp_layer != z: continue

                color = '#00ff00' if ref == "TP5" else '#ff0055'
                alpha = 0.8

                for pad in fp["pads"]:
                    w, h = pad["w_mm"], pad["h_mm"]
                    x, y = pad["x_mm"], pad["y_mm"]
                    angle = pad.get("angle_deg", 0.0)

                    if abs(angle) < 0.01:
                        rect = Rectangle((x - w / 2, y - h / 2), w, h, facecolor=color, edgecolor='white',
                                         linewidth=0.5, alpha=alpha, zorder=10)
                        ax.add_patch(rect)
                    else:
                        rad = math.radians(-angle)
                        ca, sa = math.cos(rad), math.sin(rad)
                        hw, hh = w / 2, h / 2
                        pts = [(x + dx * ca - dy * sa, y + dx * sa + dy * ca) for dx, dy in
                               [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]]
                        poly = Polygon(pts, closed=True, facecolor=color, edgecolor='white', linewidth=0.5, alpha=alpha,
                                       zorder=10)
                        ax.add_patch(poly)

                # Add text label on the largest pad
                ax.text(fp["x_mm"], fp["y_mm"], ref, fontsize=6, color='white', ha='center', va='center', weight='bold',
                        zorder=11)

        self.canvas.draw()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = TensorValidator()
    win.show()
    sys.exit(app.exec())