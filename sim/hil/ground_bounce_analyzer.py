import sys
import json
import math
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu
import matplotlib

matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Polygon, Rectangle
from matplotlib.path import Path

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QSlider, QLabel, QTabWidget
)
from PyQt6.QtCore import Qt

GEOMETRY_JSON = "pcb_geometry.json"
LAYERS = ["F_Cu", "In1_Cu", "In2_Cu", "In3_Cu", "In4_Cu", "B_Cu"]
GRID_DX_MM = 0.5
COPPER_RESISTIVITY = 1.72e-8
COPPER_THICKNESS_M = 35e-6
MAX_CURRENT = 41.4

LOW_SIDE_FETS = ["Q2", "Q4", "Q6", "Q8", "Q10", "Q12"]


class GroundBounceTab(QWidget):
    def __init__(self, geom=None):
        super().__init__()

        # Allow standalone execution
        if geom is None:
            with open(GEOMETRY_JSON) as f:
                geom = json.load(f)
        self.geom = geom

        self.tabs = QTabWidget()
        layout = QVBoxLayout(self)
        layout.addWidget(self.tabs)

        self.bounce_tab = QWidget()
        self.bounce_layout = QVBoxLayout(self.bounce_tab)

        self.highway_tab = QWidget()
        self.highway_layout = QVBoxLayout(self.highway_tab)

        # --- Controls ---
        self.ctrl_layout = QHBoxLayout()
        self.lbl_current = QLabel("Phase Current: 0.0 A")
        self.lbl_current.setStyleSheet("font-size: 20px; font-weight: bold; color: #00f0ff;")

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, int(MAX_CURRENT * 10))
        self.slider.setValue(400)
        self.slider.valueChanged.connect(self.update_current)

        self.ctrl_layout.addWidget(self.lbl_current)
        self.ctrl_layout.addWidget(self.slider)
        self.bounce_layout.addLayout(self.ctrl_layout)

        # --- Plots ---
        plt.style.use('dark_background')

        self.fig = Figure(figsize=(18, 8.5), facecolor='#111111')
        self.fig.subplots_adjust(left=0.02, right=0.92, top=0.92, bottom=0.05, wspace=0.05, hspace=0.15)
        self.canvas = FigureCanvas(self.fig)
        self.bounce_layout.addWidget(self.canvas)

        self.highway_fig = Figure(figsize=(18, 8.5), facecolor='#111111')
        self.highway_fig.subplots_adjust(left=0.02, right=0.92, top=0.92, bottom=0.05, wspace=0.05, hspace=0.15)
        self.highway_canvas = FigureCanvas(self.highway_fig)
        self.highway_layout.addWidget(self.highway_canvas)

        self.tabs.addTab(self.bounce_tab, "3D Bounce Heatmap")
        self.tabs.addTab(self.highway_tab, "Solder Highways by Net")

        print("[*] Building 3D Grid and Factorizing Matrix...")
        self.setup_solver()
        print("[*] Ready. Move the slider to update.")
        self.update_current()

    def setup_solver(self):
        o = self.geom["outline"]
        self.bx, self.by, self.bw, self.bh = o["x_mm"], o["y_mm"], o["w_mm"], o["h_mm"]
        self.nx = int(math.ceil(self.bw / GRID_DX_MM))
        self.ny = int(math.ceil(self.bh / GRID_DX_MM))
        self.nz = 6

        nx, ny, nz = self.nx, self.ny, self.nz
        bx, by, bw, bh = self.bx, self.by, self.bw, self.bh

        self.tensor = [[[{"copper": 0.0, "vias": 0, "tags": []}
                         for x in range(nx)] for y in range(ny)] for z in range(nz)]

        self.solder_mask = np.zeros((nz, ny, nx), dtype=bool)

        # 1. Rasterize Copper Pours
        for p in self.geom.get("pours", []):
            layer = p.get("layer")
            if layer not in LAYERS: continue
            if "GND" not in p.get("net_name", "").upper(): continue
            z = LAYERS.index(layer)
            path = Path(p["points"])
            xs = [pt[0] for pt in p["points"]]
            ys = [pt[1] for pt in p["points"]]
            j0 = max(0, int((min(xs) - bx) / GRID_DX_MM))
            j1 = min(nx - 1, int((max(xs) - bx) / GRID_DX_MM))
            i0 = max(0, int((min(ys) - by) / GRID_DX_MM))
            i1 = min(ny - 1, int((max(ys) - by) / GRID_DX_MM))
            if j1 < j0 or i1 < i0: continue
            xx, yy = np.meshgrid(bx + (np.arange(j0, j1 + 1) + 0.5) * GRID_DX_MM,
                                 by + (np.arange(i0, i1 + 1) + 0.5) * GRID_DX_MM)
            test_pts = np.column_stack((xx.flatten(), yy.flatten()))
            inside = path.contains_points(test_pts).reshape((i1 - i0 + 1, j1 - j0 + 1))
            for i_local, i_global in enumerate(range(i0, i1 + 1)):
                for j_local, j_global in enumerate(range(j0, j1 + 1)):
                    if inside[i_local, j_local]:
                        self.tensor[z][i_global][j_global]["copper"] = 1.0

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
                    self.tensor[z][i][j]["copper"] = 1.0

        # 3. Rasterize Vias
        for v in self.geom.get("vias", []):
            j = int((v["x_mm"] - bx) / GRID_DX_MM)
            i = int((v["y_mm"] - by) / GRID_DX_MM)
            if 0 <= i < ny and 0 <= j < nx:
                for z in range(nz):
                    self.tensor[z][i][j]["vias"] += 1

        # 3.5 Rasterize Solder Mask Openings
        for m in self.geom.get("mask_openings", []):
            layer = m.get("layer")
            if layer == "F_Mask":
                z_target = 0
            elif layer == "B_Mask":
                z_target = 5
            else:
                continue

            path = Path(m["points"])
            xs = [pt[0] for pt in m["points"]]
            ys = [pt[1] for pt in m["points"]]
            j0 = max(0, int((min(xs) - bx) / GRID_DX_MM))
            j1 = min(nx - 1, int((max(xs) - bx) / GRID_DX_MM))
            i0 = max(0, int((min(ys) - by) / GRID_DX_MM))
            i1 = min(ny - 1, int((max(ys) - by) / GRID_DX_MM))
            if j1 < j0 or i1 < i0: continue

            xx, yy = np.meshgrid(bx + (np.arange(j0, j1 + 1) + 0.5) * GRID_DX_MM,
                                 by + (np.arange(i0, i1 + 1) + 0.5) * GRID_DX_MM)
            test_pts = np.column_stack((xx.flatten(), yy.flatten()))
            inside = path.contains_points(test_pts).reshape((i1 - i0 + 1, j1 - j0 + 1))

            for i_local, i_global in enumerate(range(i0, i1 + 1)):
                for j_local, j_global in enumerate(range(j0, j1 + 1)):
                    if inside[i_local, j_local]:
                        self.solder_mask[z_target, i_global, j_global] = True

        # 4. Tag Components
        def tag_pad_area(fp, ref, z_pad):
            for pad in fp["pads"]:
                w, h = pad["w_mm"], pad["h_mm"]
                px, py = pad["x_mm"], pad["y_mm"]
                j0 = max(0, int((px - w / 2 - bx) / GRID_DX_MM))
                j1 = min(nx - 1, int((px + w / 2 - bx) / GRID_DX_MM))
                i0 = max(0, int((py - h / 2 - by) / GRID_DX_MM))
                i1 = min(ny - 1, int((py + h / 2 - by) / GRID_DX_MM))
                for i in range(i0, i1 + 1):
                    for j in range(j0, j1 + 1):
                        if self.tensor[z_pad][i][j]["copper"] > 0.5:
                            self.tensor[z_pad][i][j]["tags"].append(ref)

        for fp in self.geom.get("footprints", []):
            ref = fp["ref"].upper()
            if not fp.get("pads"): continue
            z_pad = 0 if fp.get("layer") == "F_Cu" else 5
            if ref == "TP5":
                tag_pad_area(fp, "TP5", z_pad)
            elif ref in LOW_SIDE_FETS:
                tag_pad_area(fp, f"FET_{ref}", z_pad)

        # 5. Build Solver Matrix
        N = nz * ny * nx
        rows, cols, vals = [], [], []

        R_sq = COPPER_RESISTIVITY / COPPER_THICKNESS_M
        G_adj = 1.0 / R_sq
        G_via = 1000.0

        def k(z, i, j):
            return z * (ny * nx) + i * nx + j

        self.tp5_nodes = []
        self.fet_nodes = []

        SOLDER_BOOST = 3.0
        VIA_BOOST = 4.0

        for z in range(nz):
            for i in range(ny):
                for j in range(nx):
                    cell = self.tensor[z][i][j]
                    if "TP5" in cell["tags"]: self.tp5_nodes.append(k(z, i, j))
                    if any(t.startswith("FET_") for t in cell["tags"]):
                        self.fet_nodes.append(k(z, i, j))

                    kk = k(z, i, j)
                    diag = 0.0
                    is_solder = self.solder_mask[z, i, j]

                    if cell["copper"] > 0.1:
                        if j > 0 and self.tensor[z][i][j - 1]["copper"] > 0.1:
                            boost = SOLDER_BOOST if (is_solder and self.solder_mask[z, i, j - 1]) else 1.0
                            g = G_adj * boost
                            rows.append(kk);
                            cols.append(k(z, i, j - 1));
                            vals.append(-g);
                            diag += g

                        if j < nx - 1 and self.tensor[z][i][j + 1]["copper"] > 0.1:
                            boost = SOLDER_BOOST if (is_solder and self.solder_mask[z, i, j + 1]) else 1.0
                            g = G_adj * boost
                            rows.append(kk);
                            cols.append(k(z, i, j + 1));
                            vals.append(-g);
                            diag += g

                        if i > 0 and self.tensor[z][i - 1][j]["copper"] > 0.1:
                            boost = SOLDER_BOOST if (is_solder and self.solder_mask[z, i - 1, j]) else 1.0
                            g = G_adj * boost
                            rows.append(kk);
                            cols.append(k(z, i - 1, j));
                            vals.append(-g);
                            diag += g

                        if i < ny - 1 and self.tensor[z][i + 1][j]["copper"] > 0.1:
                            boost = SOLDER_BOOST if (is_solder and self.solder_mask[z, i + 1, j]) else 1.0
                            g = G_adj * boost
                            rows.append(kk);
                            cols.append(k(z, i + 1, j));
                            vals.append(-g);
                            diag += g

                    if cell["vias"] > 0:
                        g_v = G_via * cell["vias"]
                        if is_solder:
                            g_v *= VIA_BOOST
                        for z_target in range(nz):
                            if z_target == z: continue
                            if self.tensor[z_target][i][j]["copper"] > 0.1:
                                rows.append(kk)
                                cols.append(k(z_target, i, j))
                                vals.append(-g_v)
                                diag += g_v

                    if cell["copper"] < 0.1 and cell["vias"] == 0:
                        diag += 1e-9
                    else:
                        diag += 1e-6

                    rows.append(kk)
                    cols.append(kk)
                    vals.append(diag)

        self.fet_nodes = list(set(self.fet_nodes))
        self.tp5_nodes = list(set(self.tp5_nodes))

        print(f"[*] Caching LU Factorization for {N} nodes...")
        A = sp.csr_matrix((vals, (rows, cols)), shape=(N, N))
        self.lu = splu(A.tocsc())
        self.N = N

        self._build_bounce_plots(bx, by, bw, bh, nx, ny, nz)
        self._build_highway_plots(bx, by, bw, bh, nx, ny, nz)

        self.mask = np.zeros((nz, ny, nx), dtype=bool)
        for z in range(nz):
            for i in range(ny):
                for j in range(nx):
                    cell = self.tensor[z][i][j]
                    if cell["copper"] < 0.1 and cell["vias"] == 0: self.mask[z, i, j] = True

    def _build_bounce_plots(self, bx, by, bw, bh, nx, ny, nz):
        self.axes = []
        self.ims = []
        self.V_display = [np.full((ny, nx), np.nan) for _ in range(nz)]
        self.cmap = plt.cm.jet.copy()
        self.cmap.set_bad('#111111')

        for z in range(nz):
            ax = self.fig.add_subplot(2, 3, z + 1)
            ax.set_facecolor('#111111')
            ax.set_aspect('equal')
            ax.axis('off')
            ax.set_xlim(bx - 2, bx + bw + 2)
            ax.set_ylim(by + bh + 2, by - 2)
            ax.set_title(f"{LAYERS[z]}", color='white', fontweight='bold', fontsize=14)
            im = ax.imshow(self.V_display[z], extent=[bx, bx + nx * GRID_DX_MM, by + ny * GRID_DX_MM, by],
                           origin='upper', cmap=self.cmap, alpha=1.0, interpolation='nearest', vmin=0, vmax=50)
            self.axes.append(ax)
            self.ims.append(im)

            for fp in self.geom.get("footprints", []):
                ref = fp["ref"].upper()
                if ref != "TP5" and ref not in LOW_SIDE_FETS: continue
                fp_layer = 0 if fp.get("layer") == "F_Cu" else 5
                if fp_layer != z: continue
                edge_color = '#00ff00' if ref == "TP5" else 'white'
                lw = 1.5 if ref == "TP5" else 0.8
                for pad in fp["pads"]:
                    w, h = pad["w_mm"], pad["h_mm"]
                    x, y = pad["x_mm"], pad["y_mm"]
                    angle = pad.get("angle_deg", 0.0)
                    if abs(angle) < 0.01:
                        rect = Rectangle((x - w / 2, y - h / 2), w, h, facecolor='none', edgecolor=edge_color,
                                         linewidth=lw, alpha=0.8, zorder=10)
                        ax.add_patch(rect)
                    else:
                        rad = math.radians(-angle)
                        ca, sa = math.cos(rad), math.sin(rad)
                        hw, hh = w / 2, h / 2
                        pts = [(x + dx * ca - dy * sa, y + dx * sa + dy * ca) for dx, dy in
                               [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]]
                        poly = Polygon(pts, closed=True, facecolor='none', edgecolor=edge_color, linewidth=lw,
                                       alpha=0.8, zorder=10)
                        ax.add_patch(poly)
                if ref == "TP5":
                    ax.text(fp["x_mm"] + 2, fp["y_mm"] + 2, "TP5", color='#00ff00', fontsize=8, fontweight='bold',
                            zorder=11)

        self.cbar = self.fig.colorbar(self.ims[0], ax=self.axes, fraction=0.02, pad=0.02)
        self.cbar.set_label("Max Ground Bounce (mV)", color="white", fontsize=14)
        self.cbar.ax.yaxis.set_tick_params(color="white")
        plt.setp(self.cbar.ax.get_yticklabels(), color="white")

    def _build_highway_plots(self, bx, by, bw, bh, nx, ny, nz):
        for z in range(nz):
            ax = self.highway_fig.add_subplot(2, 3, z + 1)
            ax.set_facecolor('#111111')
            ax.set_aspect('equal')
            ax.axis('off')
            ax.set_xlim(bx - 2, bx + bw + 2)
            ax.set_ylim(by + bh + 2, by - 2)
            ax.set_title(f"{LAYERS[z]}", color='white', fontweight='bold', fontsize=14)

            copper = np.zeros((ny, nx))
            for i in range(ny):
                for j in range(nx):
                    if self.tensor[z][i][j]["copper"] > 0.1: copper[i, j] = 1
            ax.imshow(copper, extent=[bx, bx + nx * GRID_DX_MM, by + ny * GRID_DX_MM, by],
                      origin='upper', cmap='Blues', alpha=0.2, interpolation='nearest')

            mask_layer = "F_Mask" if z == 0 else ("B_Mask" if z == 5 else None)
            if mask_layer:
                for m in self.geom.get("mask_openings", []):
                    if m.get("layer") == mask_layer:
                        pts = m["points"]
                        net = m.get("net_name", "UNKNOWN")

                        if "GND" in net.upper():
                            color = '#ff3333'
                        elif "BATT" in net.upper():
                            color = '#ffaa00'
                        elif net.upper().endswith("U"):
                            color = '#00ff00'
                        elif net.upper().endswith("V"):
                            color = '#00ffff'
                        elif net.upper().endswith("W"):
                            color = '#ff00ff'
                        else:
                            color = '#ffff00'

                        poly = Polygon(pts, closed=True, facecolor=color, edgecolor='white', linewidth=0.5, alpha=0.8)
                        ax.add_patch(poly)

                        if len(pts) > 3:
                            cx = sum(p[0] for p in pts) / len(pts)
                            cy = sum(p[1] for p in pts) / len(pts)
                            ax.text(cx, cy, net, color='white', fontsize=8, ha='center', va='center',
                                    fontweight='bold', bbox=dict(facecolor='black', alpha=0.6, pad=1, edgecolor='none'))

            rect = Rectangle((bx, by), bw, bh, fill=False, edgecolor='#555555', linewidth=1, linestyle='--')
            ax.add_patch(rect)

        self.highway_fig.suptitle("Identified Solder Highways by Net", color='white', fontsize=18, fontweight='bold')

    def update_current(self):
        current_a = self.slider.value() / 10.0
        self.lbl_current.setText(f"Phase Current: {current_a:.1f} A")

        b = np.zeros(self.N)
        i_per_fet = current_a / len(self.fet_nodes)
        for node in self.fet_nodes: b[node] += i_per_fet
        i_per_tp5 = current_a / len(self.tp5_nodes)
        for node in self.tp5_nodes: b[node] -= i_per_tp5

        V = self.lu.solve(b)
        v_ref = np.mean([V[n] for n in self.tp5_nodes])
        V = V - v_ref

        V_tensor = V.reshape((self.nz, self.ny, self.nx))
        V_mV = V_tensor * 1000.0
        V_mV[self.mask] = np.nan

        vmax = np.nanmax(V_mV)
        if np.isnan(vmax) or vmax < 1.0: vmax = 1.0

        for z in range(self.nz):
            self.ims[z].set_data(V_mV[z])
            self.ims[z].set_clim(0, vmax)

        self.cbar.mappable.set_clim(0, vmax)
        self.fig.suptitle(f"3D Tensor Ground Bounce (Current: {current_a:.1f} A)", color='white', fontsize=18,
                          fontweight='bold')
        self.canvas.draw_idle()


# --- Standalone runner ---
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = QMainWindow()
    win.setWindowTitle("Standalone GND Bounce")
    win.resize(1800, 950)
    win.setCentralWidget(GroundBounceTab())
    win.show()
    sys.exit(app.exec())