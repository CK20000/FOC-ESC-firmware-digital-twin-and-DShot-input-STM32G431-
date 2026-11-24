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
from matplotlib.collections import PatchCollection
from matplotlib.colors import LogNorm
from matplotlib.cm import ScalarMappable
from matplotlib.ticker import FormatStrFormatter
from matplotlib.path import Path
import re
import scipy.sparse as sp
from scipy.sparse.linalg import splu

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QSlider, QLabel, QTabWidget, QCheckBox, QPushButton
)
from PyQt6.QtCore import Qt, QRectF, QTimer

import pyqtgraph as pg
from pyqtgraph import ColorBarItem

# --- GLOBAL CONSTANTS ---
GEOMETRY_JSON = "pcb_geometry.json"
NETLIST_JSON = "digital_twin_netlist.json"
MAX_CURRENT = 41.55
LAYERS = ["F_Cu", "In1_Cu", "In2_Cu", "In3_Cu", "In4_Cu", "B_Cu"]
COPPER_RESISTIVITY = 1.72e-8
COPPER_THICKNESS_M = 35e-6
RDS_ON_PER_FET = 0.008
FETS_PARALLEL = 2
R_SHUNT = 0.002

GRID_DX_MM = 0.5
LOW_SIDE_FETS = ["Q2", "Q4", "Q6", "Q8", "Q10", "Q12"]

# Transient physics
RHO_CU = 8960.0
CP_CU = 385.0
RHO_FR4 = 1850.0
CP_FR4 = 1000.0
T_CU_MM = 0.035
T_FR4_MM = 1.6 - (2 * 0.035)

pg.setConfigOptions(antialias=False, background='#050505', foreground='white')

STYLESHEET = """
QMainWindow, QWidget { background-color: #050505; color: #e8e8e8; }
QTabWidget::pane { border: 1px solid #333333; background-color: #050505; }
QTabBar::tab { background-color: #111111; color: #888888; padding: 8px 20px; border: 1px solid #333333; border-bottom: none; font-size: 10pt; }
QTabBar::tab:selected { background-color: #050505; color: #ffffff; border-top: 2px solid #ffb733; }
QTabBar::tab:hover { background-color: #222222; }
QLabel { color: #e8e8e8; }
QCheckBox { color: #e8e8e8; spacing: 8px; }
QSlider::groove:horizontal { border: 1px solid #333; height: 6px; background: #111; margin: 2px 0; border-radius: 3px; }
QSlider::handle:horizontal { background: #ffb733; border: 1px solid #ffb733; width: 14px; margin: -5px 0; border-radius: 7px; }
QSlider::handle:horizontal:hover { background: #ffcc66; }
QPushButton { background-color: #222222; color: #ffffff; border: 1px solid #555555; border-radius: 4px; }
QPushButton:hover { background-color: #333333; }
QPushButton:checked { background-color: #ffb733; color: #000000; border: 1px solid #ffb733; }
"""


def is_power_path(net):
    if not net: return False
    n = "".join(c for c in net.upper() if c.isalnum())
    if any(n.endswith(p) for p in ("U", "V", "W", "TU", "TV", "TW", "BATT", "PHASE")): return True
    if any(b in n for b in ("SENSE", "PWM", "GATE", "DRV", "BOOT", "3V3", "5V", "12V")): return False
    return False


def is_gnd(net): return "GND" in net.upper()


def calc_poly_resistance(pts):
    if len(pts) < 3: return 999.0
    xs = [pt[0] for pt in pts]
    ys = [pt[1] for pt in pts]
    L_m = max(max(xs) - min(xs), max(ys) - min(ys)) / 1000.0
    W_m = min(max(xs) - min(xs), max(ys) - min(ys)) / 1000.0
    if W_m <= 0 or L_m <= 0: return 999.0
    return (COPPER_RESISTIVITY * L_m / (W_m * COPPER_THICKNESS_M))


def get_net_resistance(geom, net_type):
    min_r = 999.0
    for p in geom.get("pours", []):
        net = p.get("net_name") or ""
        n = "".join(c for c in net.upper() if c.isalnum())
        match = False
        if net_type == "BATT" and "BATT" in n:
            match = True
        elif net_type == "U" and n.endswith("U") and not n.endswith("TU") and "GND" not in n:
            match = True
        elif net_type == "V" and n.endswith("V") and not n.endswith("TV") and "GND" not in n:
            match = True
        elif net_type == "W" and n.endswith("W") and not n.endswith("TW") and "GND" not in n:
            match = True
        elif net_type == "TU" and n.endswith("TU"):
            match = True
        elif net_type == "TV" and n.endswith("TV"):
            match = True
        elif net_type == "TW" and n.endswith("TW"):
            match = True
        if match:
            r = calc_poly_resistance(p.get("points", []))
            if r < min_r: min_r = r
    return min_r if min_r != 999.0 else 0.0


# ==============================================================================
# THERMAL SOLVER
# ==============================================================================
class ThermalSolver:
    def __init__(self, geom, powers, h_conv=15.0, h_sink=0.0):
        self.geom = geom
        self.powers = powers
        self.h_conv = h_conv
        self.h_sink = h_sink

        self.T_CU_MM = 0.035
        self.Z_GAPS_MM = [0.1, 0.45, 0.45, 0.45, 0.1]
        self.K_CU = 385.0
        self.K_FR4 = 0.30
        self.LAYERS = ["F_Cu", "In1_Cu", "In2_Cu", "In3_Cu", "In4_Cu", "B_Cu"]
        self.COV_BG = {L: 0.1 for L in self.LAYERS}

        self.VIA_DIA_MM = 0.30
        self.VIA_PLATING_UM = 25.0
        self.G_VIA = [self._gvia(z) for z in self.Z_GAPS_MM]
        self.AMBIENT_C = 25.0

        o = self.geom["outline"]
        self.bx, self.by, self.bw, self.bh = o["x_mm"], o["y_mm"], o["w_mm"], o["h_mm"]
        self.dx = 0.5
        self.nx = int(math.ceil(self.bw / self.dx))
        self.ny = int(math.ceil(self.bh / self.dx))
        self.N = self.nx * self.ny
        self.M = 6 * self.N

        self.covs = {L: self._rasterize_layer(L) for L in self.LAYERS}
        self.via_cnt = self._rasterize_vias()
        self.sink_mask = self._build_heatsink_mask()

        self.A, self.G_conv, self.G_sink = self._assemble_matrix()
        self.lu = splu(self.A)

    def _gvia(self, gap_mm):
        return (self.K_CU * (math.pi * self.VIA_DIA_MM * 1e-3 * self.VIA_PLATING_UM * 1e-6) / (gap_mm * 1e-3))

    def _rasterize_layer(self, layer):
        cov = np.full((self.ny, self.nx), self.COV_BG[layer], dtype=np.float64)

        step = self.dx / 3.0
        for t in self.geom.get("tracks", []):
            if t["layer"] != layer: continue
            x1, y1, x2, y2, w = t["x1"], t["y1"], t["x2"], t["y2"], t["width_mm"]
            length = math.hypot(x2 - x1, y2 - y1)
            if length < 1e-6: continue
            n = max(2, int(length / step))
            ds = length / n
            contrib = w * ds / (self.dx * self.dx)
            for k in range(n + 1):
                f = k / n
                j = int((x1 + (x2 - x1) * f - self.bx) / self.dx)
                i = int((y1 + (y2 - y1) * f - self.by) / self.dx)
                if 0 <= i < self.ny and 0 <= j < self.nx:
                    cov[i, j] = min(1.0, cov[i, j] + contrib)

        for p in self.geom.get("pours", []):
            if p["layer"] != layer: continue
            poly = p["points"]
            if len(poly) < 3: continue
            path = Path(poly)
            xs = [pt[0] for pt in poly]
            ys = [pt[1] for pt in poly]
            j0 = max(0, int((min(xs) - self.bx) / self.dx))
            j1 = min(self.nx - 1, int((max(xs) - self.bx) / self.dx))
            i0 = max(0, int((min(ys) - self.by) / self.dx))
            i1 = min(self.ny - 1, int((max(ys) - self.by) / self.dx))
            if j1 < j0 or i1 < i0: continue
            xx, yy = np.meshgrid(self.bx + (np.arange(j0, j1 + 1) + 0.5) * self.dx,
                                 self.by + (np.arange(i0, i1 + 1) + 0.5) * self.dx)
            pts = np.column_stack((xx.flatten(), yy.flatten()))
            inside = path.contains_points(pts).reshape((i1 - i0 + 1, j1 - j0 + 1))
            cov[i0:i1 + 1, j0:j1 + 1][inside] = np.maximum(cov[i0:i1 + 1, j0:j1 + 1][inside], 0.95)

        for fp in self.geom.get("footprints", []):
            for p in fp.get("pads", []):
                x0 = p["x_mm"] - p["w_mm"] / 2
                x1 = p["x_mm"] + p["w_mm"] / 2
                y0 = p["y_mm"] - p["h_mm"] / 2
                y1 = p["y_mm"] + p["h_mm"] / 2
                j0 = max(0, int((x0 - self.bx) / self.dx))
                j1 = min(self.nx - 1, int((x1 - self.bx) / self.dx))
                i0 = max(0, int((y0 - self.by) / self.dx))
                i1 = min(self.ny - 1, int((y1 - self.by) / self.dx))
                if j1 < j0 or i1 < i0: continue
                n = (j1 - j0 + 1) * (i1 - i0 + 1)
                add = 0.5 * p["w_mm"] * p["h_mm"] / (self.dx * self.dx * n)
                cov[i0:i1 + 1, j0:j1 + 1] = np.minimum(1.0, cov[i0:i1 + 1, j0:j1 + 1] + add)
        return np.clip(cov, 0.0, 1.0)

    def _rasterize_vias(self):
        vc = np.zeros((self.ny, self.nx), dtype=np.int32)
        for v in self.geom.get("vias", []):
            j = int((v["x_mm"] - self.bx) / self.dx)
            i = int((v["y_mm"] - self.by) / self.dx)
            if 0 <= i < self.ny and 0 <= j < self.nx: vc[i, j] += 1
        return vc

    def _build_power_grids(self):
        Q_top = np.zeros((self.ny, self.nx))
        Q_bot = np.zeros((self.ny, self.nx))

        for fp in self.geom["footprints"]:
            ref = fp["ref"].upper()
            if ref not in self.powers:
                continue
            p = self.powers[ref]
            pads = fp.get("pads", [])
            if not pads:
                continue

            is_fet = ref.startswith("Q") or "AOD" in (fp.get("value", "") or "").upper()
            is_shunt = ref in ("R22", "R23") or "4020" in (fp.get("value", "") or "")

            target = Q_top if fp.get("layer") == "F_Cu" else Q_bot

            if is_fet:
                pads = [max(pads, key=lambda pp: pp["w_mm"] * pp["h_mm"])]
                p_per_pad = p / len(pads)
                for pp in pads:
                    x0 = pp["x_mm"] - pp["w_mm"] / 2
                    x1 = pp["x_mm"] + pp["w_mm"] / 2
                    y0 = pp["y_mm"] - pp["h_mm"] / 2
                    y1 = pp["y_mm"] + pp["h_mm"] / 2
                    if x1 - x0 < self.dx: cx = (x0 + x1) / 2; x0, x1 = cx - self.dx / 2, cx + self.dx / 2
                    if y1 - y0 < self.dx: cy = (y0 + y1) / 2; y0, y1 = cy - self.dx / 2, cy + self.dx / 2
                    j0 = max(0, int((x0 - self.bx) / self.dx))
                    j1 = min(self.nx - 1, int((x1 - self.bx) / self.dx))
                    i0 = max(0, int((y0 - self.by) / self.dx))
                    i1 = min(self.ny - 1, int((y1 - self.by) / self.dx))
                    n = (j1 - j0 + 1) * (i1 - i0 + 1)
                    if n > 0: target[i0:i1 + 1, j0:j1 + 1] += p_per_pad / n

            elif is_shunt:
                x0 = min(pp["x_mm"] - pp["w_mm"] / 2 for pp in pads)
                x1 = max(pp["x_mm"] + pp["w_mm"] / 2 for pp in pads)
                y0 = min(pp["y_mm"] - pp["h_mm"] / 2 for pp in pads)
                y1 = max(pp["y_mm"] + pp["h_mm"] / 2 for pp in pads)

                j0 = max(0, int((x0 - self.bx) / self.dx))
                j1 = min(self.nx - 1, int((x1 - self.bx) / self.dx))
                i0 = max(0, int((y0 - self.by) / self.dx))
                i1 = min(self.ny - 1, int((y1 - self.by) / self.dx))
                n = (j1 - j0 + 1) * (i1 - i0 + 1)
                if n > 0:
                    target[i0:i1 + 1, j0:j1 + 1] += p / n
            else:
                p_per_pad = p / len(pads)
                for pp in pads:
                    x0 = pp["x_mm"] - pp["w_mm"] / 2
                    x1 = pp["x_mm"] + pp["w_mm"] / 2
                    y0 = pp["y_mm"] - pp["h_mm"] / 2
                    y1 = pp["y_mm"] + pp["h_mm"] / 2
                    if x1 - x0 < self.dx: cx = (x0 + x1) / 2; x0, x1 = cx - self.dx / 2, cx + self.dx / 2
                    if y1 - y0 < self.dx: cy = (y0 + y1) / 2; y0, y1 = cy - self.dx / 2, cy + self.dx / 2
                    j0 = max(0, int((x0 - self.bx) / self.dx))
                    j1 = min(self.nx - 1, int((x1 - self.bx) / self.dx))
                    i0 = max(0, int((y0 - self.by) / self.dx))
                    i1 = min(self.ny - 1, int((y1 - self.by) / self.dx))
                    n = (j1 - j0 + 1) * (i1 - i0 + 1)
                    if n > 0: target[i0:i1 + 1, j0:j1 + 1] += p_per_pad / n

        return Q_top, Q_bot

    def _build_heatsink_mask(self):
        mask = np.zeros((self.ny, self.nx), dtype=bool)
        fet_pads = []
        for fp in self.geom["footprints"]:
            if fp["ref"].upper().startswith("Q"):
                for p in fp.get("pads", []): fet_pads.append((p["x_mm"], p["y_mm"], p["w_mm"], p["h_mm"]))
        if not fet_pads: return mask
        x0s = [p[0] - p[2] / 2 for p in fet_pads]
        x1s = [p[0] + p[2] / 2 for p in fet_pads]
        y0s = [p[1] - p[3] / 2 for p in fet_pads]
        y1s = [p[1] + p[3] / 2 for p in fet_pads]
        min_x, max_x = min(x0s) - 2.0, max(x1s) + 2.0
        min_y, max_y = min(y0s) - 2.0, max(y1s) + 2.0
        j0 = max(0, int((min_x - self.bx) / self.dx))
        j1 = min(self.nx - 1, int((max_x - self.bx) / self.dx))
        i0 = max(0, int((min_y - self.by) / self.dx))
        i1 = min(self.ny - 1, int((max_y - self.by) / self.dx))
        mask[i0:i1 + 1, j0:j1 + 1] = True
        return mask

    def _assemble_matrix(self):
        dx_m = self.dx * 1e-3
        t_cu = self.T_CU_MM * 1e-3
        G_sheet = {L: self.covs[L] * self.K_CU * t_cu for L in self.LAYERS}
        G_fr4 = [self.K_FR4 * dx_m * dx_m / (z * 1e-3) for z in self.Z_GAPS_MM]
        G_coup = [G_fr4[i] + self.via_cnt * self.G_VIA[i] for i in range(5)]
        G_conv = self.h_conv * dx_m * dx_m
        G_sink = self.h_sink * dx_m * dx_m

        rows, cols, vals = [], [], []

        def k(li, i, j):
            return li * self.N + i * self.nx + j

        if self.h_sink > 0:
            G_al = 1000.0
            for i in range(self.ny):
                for j in range(self.nx):
                    if self.sink_mask[i, j]:
                        ka, kb = k(0, i, j), k(5, i, j)
                        rows.extend([ka, kb, ka, kb])
                        cols.extend([kb, ka, ka, kb])
                        vals.extend([G_al, G_al, -G_al, -G_al])

        for li, L in enumerate(self.LAYERS):
            G = G_sheet[L]
            Gx = np.minimum(G[:, :-1], G[:, 1:])
            Gy = np.minimum(G[:-1, :], G[1:, :])
            for i in range(self.ny):
                for j in range(self.nx):
                    kk = k(li, i, j)
                    diag = 0.0
                    if j < self.nx - 1: rows.append(kk); cols.append(k(li, i, j + 1)); vals.append(Gx[i, j]); diag -= \
                    Gx[i, j]
                    if j > 0: rows.append(kk); cols.append(k(li, i, j - 1)); vals.append(Gx[i, j - 1]); diag -= Gx[
                        i, j - 1]
                    if i < self.ny - 1: rows.append(kk); cols.append(k(li, i + 1, j)); vals.append(Gy[i, j]); diag -= \
                    Gy[i, j]
                    if i > 0: rows.append(kk); cols.append(k(li, i - 1, j)); vals.append(Gy[i - 1, j]); diag -= Gy[
                        i - 1, j]
                    if li in (0, 5):
                        diag -= G_conv
                        if self.h_sink > 0 and self.sink_mask[i, j]: diag -= G_sink
                    rows.append(kk)
                    cols.append(kk)
                    vals.append(diag)

        for i in range(self.ny):
            for j in range(self.nx):
                for li in range(5):
                    Gc = G_coup[li][i, j]
                    ka, kb = k(li, i, j), k(li + 1, i, j)
                    rows.extend([ka, kb, ka, kb])
                    cols.extend([kb, ka, ka, kb])
                    vals.extend([Gc, Gc, -Gc, -Gc])

        A = sp.csr_matrix((vals, (rows, cols)), shape=(self.M, self.M))
        return A.tocsc(), G_conv, G_sink

    def solve(self):
        Q_L1, Q_L6 = self._build_power_grids()
        rhs = np.zeros(self.M)
        sink_flat = self.sink_mask.flatten() if self.h_sink > 0 else np.zeros(self.N)
        for li in range(6):
            if li == 0:
                rhs[li * self.N:(li + 1) * self.N] -= Q_L1.flatten() + (
                        self.G_conv + self.G_sink * sink_flat) * self.AMBIENT_C
            elif li == 5:
                rhs[li * self.N:(li + 1) * self.N] -= Q_L6.flatten() + (
                        self.G_conv + self.G_sink * sink_flat) * self.AMBIENT_C
        T = self.lu.solve(rhs)
        return [T[i * self.N:(i + 1) * self.N].reshape((self.ny, self.nx)) for i in range(6)]


# ==============================================================================
# TRANSIENT SOLVER CLASS
# ==============================================================================
class TransientSolver(ThermalSolver):
    def __init__(self, geom, powers, h_conv=15.0, h_sink=150.0):
        super().__init__(geom, powers, h_conv, h_sink)
        dx_m = self.dx * 1e-3
        vol_cu = dx_m * dx_m * (T_CU_MM * 1e-3)
        vol_fr4 = dx_m * dx_m * (T_FR4_MM * 1e-3)
        c_cu = RHO_CU * CP_CU * vol_cu
        c_fr4 = RHO_FR4 * CP_FR4 * vol_fr4
        self.C_vec = np.zeros(self.M)
        for i in range(6):
            self.C_vec[i * self.N:(i + 1) * self.N] = c_cu
            if i in (0, 5): self.C_vec[i * self.N:(i + 1) * self.N] += c_fr4 / 2.0
        self.dt = 0.5
        C_mat = sp.diags(self.C_vec)
        self.A_trans = (C_mat / self.dt) - self.A
        self.lu_trans = splu(self.A_trans.tocsc())
        self.T_current = np.full(self.M, self.AMBIENT_C)

    def step(self, powers):
        self.powers = powers
        Q_L1, Q_L6 = self._build_power_grids()
        b = np.zeros(self.M)
        sink_flat = self.sink_mask.flatten() if self.h_sink > 0 else np.zeros(self.N)
        for li in range(6):
            if li == 0:
                b[li * self.N:(li + 1) * self.N] = -Q_L1.flatten() - (
                        self.G_conv + self.G_sink * sink_flat) * self.AMBIENT_C
            elif li == 5:
                b[li * self.N:(li + 1) * self.N] = -Q_L6.flatten() - (
                        self.G_conv + self.G_sink * sink_flat) * self.AMBIENT_C
        rhs = (self.C_vec / self.dt) * self.T_current - b
        self.T_current = self.lu_trans.solve(rhs)
        return [self.T_current[i * self.N:(i + 1) * self.N].reshape((self.ny, self.nx)) for i in range(6)]


# ==============================================================================
# TAB: SIGNAL INTEGRITY
# ==============================================================================
class SignalIntegrityTab(QWidget):
    R_GATE = 10.0
    R_DRIVER = 2.0
    L_PER_MM = 1.0

    def __init__(self, geom, netlist):
        super().__init__()
        self.geom = geom
        self.netlist = netlist
        self.layout = QVBoxLayout(self)
        plt.style.use('dark_background')
        self.fig = Figure(figsize=(16, 10), facecolor='#050505')
        self.fig.subplots_adjust(left=0.08, right=0.95, top=0.90, bottom=0.08, wspace=0.25, hspace=0.35)
        self.canvas = FigureCanvas(self.fig)
        self.layout.addWidget(self.canvas)
        self._analyze()

    def _total_length(self, net_name):
        total = 0.0
        for t in self.geom.get("tracks", []):
            if (t.get("net_name") or "").upper() == net_name.upper():
                total += math.hypot(t["x2"] - t["x1"], t["y2"] - t["y1"])
        return total

    def _extract_rc_filter(self, start_net):
        r_val, c_val = None, None
        r_ref = None
        if start_net in self.netlist:
            for node in self.netlist[start_net]["nodes"]:
                if node["component"].startswith("R"):
                    r_ref = node["component"]
                    try:
                        r_val = float(re.sub(r'[^0-9.]', '', node["value"]))
                    except Exception:
                        pass
                    break
        if not r_ref: return 100.0, 2.2e-9

        other_net = None
        for net_name, data in self.netlist.items():
            if net_name == start_net: continue
            for node in data["nodes"]:
                if node["component"] == r_ref:
                    other_net = net_name
                    break
            if other_net: break
        if not other_net: return (r_val or 100.0), 2.2e-9

        if other_net in self.netlist:
            for node in self.netlist[other_net]["nodes"]:
                if node["component"].startswith("C"):
                    try:
                        v = node["value"].lower()
                        if "n" in v:
                            c_val = float(re.sub(r'[^0-9.]', '', node["value"])) * 1e-9
                        elif "u" in v:
                            c_val = float(re.sub(r'[^0-9.]', '', node["value"])) * 1e-6
                        elif "p" in v:
                            c_val = float(re.sub(r'[^0-9.]', '', node["value"])) * 1e-12
                        else:
                            c_val = float(re.sub(r'[^0-9.]', '', node["value"])) * 1e-12
                    except Exception:
                        pass
                    break
        return (r_val or 100.0), (c_val or 2.2e-9)

    def _analyze(self):
        geom, netlist = self.geom, self.netlist
        len_adc1, len_adc2 = self._total_length("ADC1"), self._total_length("ADC2")
        r_adc1, c_adc1 = self._extract_rc_filter("ADC1")
        r_adc2, c_adc2 = self._extract_rc_filter("ADC2")

        w_trace = 0.3
        R1 = COPPER_RESISTIVITY * (len_adc1 / 1000.0) / ((w_trace / 1000.0) * COPPER_THICKNESS_M)
        L1, C1 = len_adc1 * 1e-9, len_adc1 * 1e-12
        R2 = COPPER_RESISTIVITY * (len_adc2 / 1000.0) / ((w_trace / 1000.0) * COPPER_THICKNESS_M)
        L2, C2 = len_adc2 * 1e-9, len_adc2 * 1e-12

        driver_nets = ["HO1", "LO1", "HO2", "LO2", "HO3", "LO3"]
        fet_gate_nets = {}
        pattern_q = re.compile(r'^Net-\((Q\d+)-G\)$', re.IGNORECASE)
        for t in geom.get("tracks", []):
            net = t.get("net_name") or ""
            m = pattern_q.match(net)
            if m:
                q_ref = m.group(1).upper()
                if net not in fet_gate_nets:
                    fet_gate_nets[net] = {"ref": q_ref, "length": 0.0}
                fet_gate_nets[net]["length"] += math.hypot(t["x2"] - t["x1"], t["y2"] - t["y1"])

        switch_map = {
            "HO1": ["Q1", "Q2"], "LO1": ["Q3", "Q4"], "HO2": ["Q5", "Q6"],
            "LO2": ["Q7", "Q8"], "HO3": ["Q9", "Q10"], "LO3": ["Q11", "Q12"],
        }

        bar_labels, driver_lengths, fet_lengths = [], [], []
        for drv in driver_nets:
            drv_len = self._total_length(drv)
            for q_ref in switch_map.get(drv, []):
                for net, data in fet_gate_nets.items():
                    if data["ref"] == q_ref:
                        bar_labels.append(f"{drv} -> {q_ref}")
                        driver_lengths.append(drv_len)
                        fet_lengths.append(data["length"])
                        break

        t = np.linspace(0, 1000e-9, 5000)
        v_src = np.where((t >= 110e-9) & (t < 130e-9), 3.65, 1.65)

        def simulate(t, v_src, R_tr, L_tr, C_tr, R_flt, C_flt):
            v_mcu = np.zeros_like(t);
            v_mcu[0] = 1.65
            i_L, v_trace, v_mcu_node = 0.0, 1.65, 1.65
            R_MCU_LEAK, C_MCU_PIN = 10e6, 5e-12
            for i in range(1, len(t)):
                dt = t[i] - t[i - 1]
                i_filter = (v_trace - v_mcu_node) / R_flt
                di_L = (v_src[i] - v_trace - i_L * R_tr) / L_tr
                dv_trace = (i_L - i_filter) / C_tr
                i_mcu_leak = v_mcu_node / R_MCU_LEAK
                dv_mcu = (i_filter - i_mcu_leak) / (C_flt + C_MCU_PIN)
                i_L += di_L * dt;
                v_trace += dv_trace * dt;
                v_mcu_node += dv_mcu * dt
                v_mcu[i] = v_mcu_node
            return v_mcu

        v_mcu1 = simulate(t, v_src, R1, L1, C1, r_adc1, c_adc1)
        v_mcu2 = simulate(t, v_src, R2, L2, C2, r_adc2, c_adc2)

        def plot_adc(ax, t, v_src, v_mcu, title, color):
            ax.plot(t * 1e9, v_src, color='#555555', linewidth=1, linestyle=':', label='Raw Spike')
            ax.plot(t * 1e9, v_mcu, color=color, linewidth=2, label='MCU Pin (Filtered)')
            ax.set_title(title, color='#ffb733', fontweight='bold', fontsize=11)
            ax.set_xlabel("Time (ns)", color='white');
            ax.set_ylabel("Voltage (V)", color='white')
            ax.tick_params(colors='white');
            ax.grid(True, alpha=0.2);
            ax.legend(loc='upper right')
            axins = ax.inset_axes([0.55, 0.25, 0.4, 0.4])
            axins.plot(t * 1e9, v_src, color='#555555', linewidth=1, linestyle=':')
            axins.plot(t * 1e9, v_mcu, color=color, linewidth=2)
            axins.set_xlim(105, 200);
            axins.set_ylim(1.5, 2.5)
            axins.grid(True, alpha=0.2);
            axins.tick_params(colors='white', labelsize=8)
            ax.indicate_inset_zoom(axins)

        ax1 = self.fig.add_subplot(221)
        plot_adc(ax1, t, v_src, v_mcu1,
                 f"ADC1 (Phase U) - {len_adc1:.1f}mm | Filter: {r_adc1:.0f}Ω, {c_adc1 * 1e9:.1f}nF", '#ff0055')
        ax2 = self.fig.add_subplot(222)
        plot_adc(ax2, t, v_src, v_mcu2,
                 f"ADC2 (Phase W) - {len_adc2:.1f}mm | Filter: {r_adc2:.0f}Ω, {c_adc2 * 1e9:.1f}nF", '#ffcc00')

        ax3 = self.fig.add_subplot(223)
        if bar_labels:
            x = np.arange(len(bar_labels))
            ax3.bar(x, driver_lengths, color='#ffb733', label='Driver Trace', edgecolor='none')
            ax3.bar(x, fet_lengths, bottom=driver_lengths, color='#ff0055', label='FET Gate Trace', edgecolor='none')
            for i in range(len(bar_labels)):
                tot_len = driver_lengths[i] + fet_lengths[i]
                delay_ns = tot_len / (self.R_GATE + self.R_DRIVER)
                ax3.text(i, tot_len + 2, f"{delay_ns:.1f}ns", ha='center', va='bottom', color='#00f0ff', fontsize=9,
                         fontweight='bold')
            ax3.set_xticks(x);
            ax3.set_xticklabels(bar_labels, rotation=45, ha='right', color='white', fontsize=8)
            ax3.set_ylim(0, max(driver_lengths[i] + fet_lengths[i] for i in range(len(bar_labels))) * 1.15)
        ax3.set_title("Total Gate Loop Length & Switching Delay", color='white', fontweight='bold')
        ax3.set_ylabel("Length (mm)", color='white');
        ax3.tick_params(colors='white')
        ax3.grid(axis='y', alpha=0.2);
        ax3.legend(facecolor='#222222', edgecolor='#555555')

        ax4 = self.fig.add_subplot(224)
        spike_window = (t >= 100e-9) & (t <= 150e-9)
        peak_noise_1 = np.max(np.abs(v_mcu1[spike_window] - 1.65))
        peak_noise_2 = np.max(np.abs(v_mcu2[spike_window] - 1.65))
        bars = ax4.bar(["ADC1 (Phase U)", "ADC2 (Phase W)"], [peak_noise_1, peak_noise_2], color=['#ff0055', '#ffcc00'],
                       alpha=0.8, width=0.4)
        ax4.set_title("Peak Noise Voltage at MCU (After RC Filter)", color='white', fontweight='bold')
        ax4.set_ylabel("Spike Amplitude (V)", color='white');
        ax4.tick_params(colors='white');
        ax4.grid(axis='y', alpha=0.2)
        for bar in bars:
            yval = bar.get_height()
            label = f'{yval * 1000:.1f} mV' if yval < 0.1 else f'{yval:.3f} V'
            ax4.text(bar.get_x() + bar.get_width() / 2, yval + 0.005, label, ha='center', va='bottom', color='white',
                     fontsize=12, fontweight='bold')
        self.fig.suptitle("Signal Integrity & FET Switching Analysis", color='white', fontsize=18, fontweight='bold')
        self.canvas.draw()


# ==============================================================================
# TAB 1: STEADY-STATE THERMAL
# ==============================================================================
class ThermalMapTab(QWidget):
    def __init__(self, geom, solver, abs_t_min, abs_t_max):
        super().__init__()
        self.geom = geom;
        self.solver = solver
        self.abs_t_min = abs_t_min;
        self.abs_t_max = abs_t_max
        self.main_layout = QVBoxLayout(self);
        self.ctrl_layout = QHBoxLayout()
        self.slider_label = QLabel(f"Phase Current: 0.0 A")
        self.slider_label.setStyleSheet("font-size: 24px; font-weight: bold; color: #ffb733;")
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, int(MAX_CURRENT * 10));
        self.slider.setValue(0)
        self.slider.valueChanged.connect(self.on_slider_change)
        self.auto_scale_cb = QCheckBox("Auto-Scale Colors")
        self.auto_scale_cb.setChecked(True);
        self.auto_scale_cb.stateChanged.connect(self.on_slider_change)
        self.ctrl_layout.addWidget(self.slider_label);
        self.ctrl_layout.addWidget(self.slider);
        self.ctrl_layout.addWidget(self.auto_scale_cb)
        self.main_layout.addLayout(self.ctrl_layout)
        self.thermal_graphics = pg.GraphicsLayoutWidget();
        self.main_layout.addWidget(self.thermal_graphics)
        self.cmap = pg.colormap.get('turbo');
        self.turbo_lut = self.cmap.getLookupTable(nPts=256)
        o = geom["outline"]
        self.bx, self.by, self.bw, self.bh = o["x_mm"], o["y_mm"], o["w_mm"], o["h_mm"]
        self.rect = QRectF(self.bx, self.by, self.bw, self.bh)
        self.img_items = {}
        for row in range(3):
            for col in range(2):
                p = self.thermal_graphics.addPlot(row=row, col=col)
                p.setAspectLocked(True);
                p.invertY(True);
                p.hideAxis('left');
                p.hideAxis('bottom')
                p.setRange(self.rect, padding=0.05)
                L_idx = row * 2 + col;
                p.setTitle(f"{LAYERS[L_idx]} Temp")
                img = pg.ImageItem();
                p.addItem(img);
                self.img_items[L_idx] = img
        self.cbar = ColorBarItem(values=(self.abs_t_min, self.abs_t_max), colorMap=self.cmap)
        self.cbar.setImageItem([self.img_items[i] for i in range(6)])
        self.thermal_graphics.addItem(self.cbar, row=0, col=2, rowspan=3)
        self.update_ui(0.0)

    def on_slider_change(self):
        self.slider_label.setText(f"Phase Current: {self.slider.value() / 10.0:.1f} A")
        self.update_ui(self.slider.value() / 10.0)

    def update_ui(self, current_a):
        p_fet = (current_a ** 2) * RDS_ON_PER_FET * 2 / 12.0
        p_shunt = (current_a ** 2) * R_SHUNT
        for i in range(1, 13): self.solver.powers[f"Q{i}"] = p_fet
        self.solver.powers["R22"] = p_shunt;
        self.solver.powers["R23"] = p_shunt
        T_fields = self.solver.solve()
        if self.auto_scale_cb.isChecked():
            t_min = min(T.min() for T in T_fields);
            t_max = max(T.max() for T in T_fields)
            if t_max - t_min < 0.5: t_max = t_min + 0.5
        else:
            t_min, t_max = self.abs_t_min, self.abs_t_max
        self.cbar.setLevels((t_min, t_max))
        for i in range(6):
            self.img_items[i].setImage(np.ascontiguousarray(T_fields[i].T), autoLevels=False, levels=(t_min, t_max),
                                       lut=self.turbo_lut)
            self.img_items[i].setRect(self.rect)


# ==============================================================================
# TAB 2: TRANSIENT THERMAL
# ==============================================================================
class TransientThermalTab(QWidget):
    def __init__(self, geom):
        super().__init__()
        self.geom = geom
        initial_powers = {"U3": 0.183, "U5": 0.158, "U9": 0.107, "R22": 0.0, "R23": 0.0}
        for i in range(1, 13): initial_powers[f"Q{i}"] = 0.0
        self.solver = TransientSolver(self.geom, initial_powers, h_conv=15.0, h_sink=150.0)
        self.current_a = 0.0;
        self.time_s = 0.0;
        self.is_running = False
        self.layout = QVBoxLayout(self);
        self.ctrl_layout = QHBoxLayout()
        self.slider_label = QLabel("Throttle: 0.0 A")
        self.slider_label.setStyleSheet("font-size: 20px; font-weight: bold; color: #ffb733;")
        self.slider = QSlider(Qt.Orientation.Horizontal);
        self.slider.setRange(0, int(MAX_CURRENT * 10))
        self.slider.valueChanged.connect(self.on_slider)
        self.btn_play = QPushButton("▶ Play");
        self.btn_play.setCheckable(True)
        self.btn_play.setStyleSheet("font-size: 18px; padding: 5px 15px;")
        self.btn_play.clicked.connect(self.toggle_play)
        self.time_label = QLabel("Time: 0.0 s")
        self.time_label.setStyleSheet("font-size: 20px; font-weight: bold; color: #00f0ff;")
        self.auto_scale_cb = QCheckBox("Auto-Scale");
        self.auto_scale_cb.setChecked(True)
        self.ctrl_layout.addWidget(self.slider_label);
        self.ctrl_layout.addWidget(self.slider)
        self.ctrl_layout.addWidget(self.btn_play);
        self.ctrl_layout.addWidget(self.time_label);
        self.ctrl_layout.addWidget(self.auto_scale_cb)
        self.layout.addLayout(self.ctrl_layout)
        self.graphics = pg.GraphicsLayoutWidget();
        self.layout.addWidget(self.graphics)
        self.cmap = pg.colormap.get('turbo');
        self.turbo_lut = self.cmap.getLookupTable(nPts=256)
        o = geom["outline"]
        self.bx, self.by, self.bw, self.bh = o["x_mm"], o["y_mm"], o["w_mm"], o["h_mm"]
        self.rect = QRectF(self.bx, self.by, self.bw, self.bh)
        self.img_items = {}
        for row in range(2):
            for col in range(3):
                p = self.graphics.addPlot(row=row, col=col)
                p.setAspectLocked(True);
                p.invertY(True);
                p.hideAxis('left');
                p.hideAxis('bottom')
                p.setRange(self.rect, padding=0.05)
                L_idx = row * 3 + col;
                p.setTitle(f"{LAYERS[L_idx]} Temp")
                img = pg.ImageItem();
                p.addItem(img);
                self.img_items[L_idx] = img
        self.cbar = ColorBarItem(values=(25, 95), colorMap=self.cmap)
        self.cbar.setImageItem([self.img_items[i] for i in range(6)])
        self.graphics.addItem(self.cbar, row=0, col=3, rowspan=2)
        self.timer = QTimer();
        self.timer.setInterval(30);
        self.timer.timeout.connect(self.update_time)
        self.update_visuals()

    def on_slider(self):
        self.current_a = self.slider.value() / 10.0
        self.slider_label.setText(f"Throttle: {self.current_a:.1f} A")

    def toggle_play(self):
        self.is_running = self.btn_play.isChecked()
        if self.is_running:
            self.btn_play.setText("⏸ Pause");
            self.timer.start()
        else:
            self.btn_play.setText("▶ Play");
            self.timer.stop()

    def update_time(self):
        for _ in range(5): self.step_sim()
        self.time_s += 2.5;
        self.time_label.setText(f"Time: {self.time_s:.1f} s");
        self.update_visuals()

    def step_sim(self):
        p_fet = (self.current_a ** 2) * RDS_ON_PER_FET * 2 / 12.0
        p_shunt = (self.current_a ** 2) * R_SHUNT
        powers = {"U3": 0.183, "U5": 0.158, "U9": 0.107, "U2": 0.165, "U1": 0.02, "U4": 0.02}
        for i in range(1, 13): powers[f"Q{i}"] = p_fet
        powers["R22"] = p_shunt;
        powers["R23"] = p_shunt
        self.solver.step(powers)

    def update_visuals(self):
        T_fields = [
            self.solver.T_current[i * self.solver.N:(i + 1) * self.solver.N].reshape((self.solver.ny, self.solver.nx))
            for i in range(6)]
        t_min = min(T.min() for T in T_fields);
        t_max = max(T.max() for T in T_fields)
        if self.auto_scale_cb.isChecked():
            if t_max - t_min < 0.5: t_max = t_min + 0.5
            levels = (t_min, t_max)
        else:
            levels = (25.0, max(95.0, min(t_max, 120.0)))
        self.cbar.setLevels(levels)
        for i in range(6):
            self.img_items[i].setImage(np.ascontiguousarray(T_fields[i].T), autoLevels=False, levels=levels,
                                       lut=self.turbo_lut)
            self.img_items[i].setRect(self.rect)


# ==============================================================================
# TAB 3: COPPER COVERAGE
# ==============================================================================
class CopperCoverageTab(QWidget):
    def __init__(self, geom, solver):
        super().__init__()
        self.main_layout = QVBoxLayout(self);
        self.graphics = pg.GraphicsLayoutWidget();
        self.main_layout.addWidget(self.graphics)
        gray_vals = np.linspace(0, 255, 256, dtype=np.uint8)
        self.gray_lut = np.column_stack((gray_vals, gray_vals, gray_vals, np.full(256, 255, dtype=np.uint8)))
        o = geom["outline"]
        self.bx, self.by, self.bw, self.bh = o["x_mm"], o["y_mm"], o["w_mm"], o["h_mm"]
        self.rect = QRectF(self.bx, self.by, self.bw, self.bh)
        for row in range(3):
            for col in range(2):
                p = self.graphics.addPlot(row=row, col=col)
                p.setAspectLocked(True);
                p.invertY(True);
                p.hideAxis('left');
                p.hideAxis('bottom')
                p.setRange(self.rect, padding=0.05)
                L_idx = row * 2 + col;
                p.setTitle(f"{LAYERS[L_idx]} Copper")
                img = pg.ImageItem();
                p.addItem(img)
                cov = np.ascontiguousarray(solver.covs[LAYERS[L_idx]].T)
                img.setImage(cov, autoLevels=False, levels=(0, 1), lut=self.gray_lut)
                img.setRect(self.rect)


# ==============================================================================
# TAB 4: POLYGON MAPS
# ==============================================================================
class PolygonMapsTab(QWidget):
    def __init__(self, geom):
        super().__init__()
        self.geom = geom;
        self.layout = QVBoxLayout(self)
        self.fig, self.axs = plt.subplots(2, 3, figsize=(16, 8), facecolor='#050505')
        self.fig.subplots_adjust(left=0.02, right=0.90, top=0.92, bottom=0.05, wspace=0.05, hspace=0.15)
        self.canvas = FigureCanvas(self.fig);
        self.layout.addWidget(self.canvas)
        self.draw_polygons()

    def draw_polygons(self):
        pours = self.geom.get("pours", []);
        vias = self.geom.get("vias", [])
        all_power_r = []
        for p in pours:
            net = p.get("net_name") or ""
            if not is_power_path(net): continue
            r_mohms = calc_poly_resistance(p.get("points", [])) * 1000.0
            if r_mohms < 999.0: all_power_r.append(r_mohms)
        norm = LogNorm(vmin=max(min(all_power_r), 1e-2), vmax=max(all_power_r)) if all_power_r else None
        cmap = matplotlib.colormaps['turbo'];
        gnd_color = '#666666'
        o = self.geom["outline"]
        bx, by, bw, bh = o["x_mm"], o["y_mm"], o["w_mm"], o["h_mm"]
        for i, layer_name in enumerate(LAYERS):
            ax = self.axs[i // 3, i % 3]
            ax.set_facecolor('#050505');
            ax.set_aspect('equal');
            ax.axis('off')
            ax.set_xlim(bx - 2, bx + bw + 2);
            ax.set_ylim(by + bh + 2, by - 2)
            ax.set_title(layer_name, color='white', fontsize=14, pad=8, weight='bold')
            ax.add_patch(
                plt.Rectangle((bx, by), bw, bh, fill=False, edgecolor='#888888', linewidth=0.5, linestyle='--'))
            layer_polys = [p for p in pours if p.get("layer") == layer_name]
            gnd_patches = [Polygon(p["points"], closed=True) for p in layer_polys if
                           is_gnd(p.get("net_name", "")) and len(p.get("points", [])) >= 3]
            if gnd_patches: ax.add_collection(
                PatchCollection(gnd_patches, facecolor=gnd_color, edgecolor=gnd_color, linewidth=0.0))
            gnd_vias, cond_vias = [], []
            for v in vias:
                net = v.get("net_name") or ""
                patch = Circle((v["x_mm"], v["y_mm"]), radius=v["diameter_mm"] / 2.0)
                if is_gnd(net):
                    gnd_vias.append(patch)
                elif is_power_path(net):
                    cond_vias.append(patch)
            if gnd_vias: ax.add_collection(
                PatchCollection(gnd_vias, facecolor=gnd_color, edgecolor=gnd_color, linewidth=0.0))
            if cond_vias: ax.add_collection(
                PatchCollection(cond_vias, facecolor='#d4af37', edgecolor='#8a6e1c', linewidth=0.3))
            for p in layer_polys:
                net = p.get("net_name") or "UNKNOWN"
                if not is_power_path(net): continue
                pts = p.get("points", [])
                if len(pts) < 3: continue
                face_color = '#888888'
                if norm is not None:
                    r_mohms = calc_poly_resistance(pts) * 1000.0
                    if r_mohms < 999.0: face_color = cmap(norm(r_mohms))
                ax.add_patch(Polygon(pts, closed=True, facecolor=face_color, edgecolor=face_color, linewidth=0.0))
                if len(pts) > 20:
                    cx = np.mean([pt[0] for pt in pts]);
                    cy = np.mean([pt[1] for pt in pts])
                    ax.text(cx, cy, net.split("/")[-1] if "/" in net else net, color='white', fontsize=7, ha='center',
                            va='center', weight='bold',
                            bbox=dict(facecolor='black', alpha=0.7, pad=1, edgecolor='none'))
        if norm is not None:
            sm = ScalarMappable(norm=norm, cmap=cmap);
            sm.set_array([])
            cbar = self.fig.colorbar(sm, ax=self.axs, fraction=0.02, pad=0.02)
            cbar.set_label("Power Net Resistance (mΩ)", color="white", fontsize=12)
            cbar.ax.yaxis.set_tick_params(color="white")
            cbar.set_ticks(np.linspace(min(all_power_r), max(all_power_r), 5))
            cbar.ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
            plt.setp(cbar.ax.get_yticklabels(), color="white")
        self.canvas.draw()


# ==============================================================================
# TAB 5: POWER LOSS DASHBOARD
# ==============================================================================
class PowerLossTab(QWidget):
    def __init__(self, data):
        super().__init__()
        self.data = data;
        self.layout = QVBoxLayout(self)
        C_BG = '#050505';
        C_U = '#00f0ff';
        C_V = '#ff0055';
        C_W = '#ffcc00';
        C_MAX = '#ffffff'
        plt.style.use('dark_background')
        self.fig = Figure(figsize=(16, 8), facecolor=C_BG)
        self.fig.subplots_adjust(left=0.06, right=0.95, top=0.90, bottom=0.10, wspace=0.25)
        self.canvas = FigureCanvas(self.fig);
        self.layout.addWidget(self.canvas)
        ax1 = self.fig.add_subplot(131)
        components = ['Battery', 'FETs', 'Phase', 'Shunt', 'Motor']
        x = np.arange(len(components));
        width = 0.25
        u_vals = [self.data["U"][k] * 1000 for k in ("batt", "fet", "phase", "shunt", "motor")]
        v_vals = [self.data["V"][k] * 1000 for k in ("batt", "fet", "phase", "shunt", "motor")]
        w_vals = [self.data["W"][k] * 1000 for k in ("batt", "fet", "phase", "shunt", "motor")]
        ax1.bar(x - width, u_vals, width, label='Phase U', color=C_U, alpha=0.85, edgecolor='none')
        ax1.bar(x, v_vals, width, label='Phase V', color=C_V, alpha=0.85, edgecolor='none')
        ax1.bar(x + width, w_vals, width, label='Phase W', color=C_W, alpha=0.85, edgecolor='none')
        ax1.set_title("Resistance Breakdown (mΩ)", color='white', fontsize=12, fontweight='bold')
        ax1.set_ylabel("Resistance (mΩ)", color='white');
        ax1.set_xticks(x);
        ax1.set_xticklabels(components, color='white', fontsize=10)
        ax1.tick_params(axis='y', colors='white');
        ax1.grid(axis='y', alpha=0.15, color='white');
        ax1.legend(facecolor='#222222', edgecolor='#555555')
        ax2 = self.fig.add_subplot(132);
        ax3 = self.fig.add_subplot(133)
        currents = np.linspace(0, 60, 100)
        for ax, metric, ylabel, title in [(ax2, "total", "Voltage Drop (mV)", "Voltage Drop vs Current"),
                                          (ax3, "total", "Power Dissipated (W)", "Power Loss vs Current")]:
            if metric == "total":
                if title.startswith("Voltage"):
                    u, v, w = currents * (self.data["U"]["total"] * 1000), currents * (
                                self.data["V"]["total"] * 1000), currents * (self.data["W"]["total"] * 1000)
                else:
                    u, v, w = (currents ** 2) * self.data["U"]["total"], (currents ** 2) * self.data["V"]["total"], (
                                currents ** 2) * self.data["W"]["total"]
            ax.plot(currents, u, color=C_U, linewidth=2.5, label='Phase U')
            ax.plot(currents, v, color=C_V, linewidth=2.5, label='Phase V')
            ax.plot(currents, w, color=C_W, linewidth=2.5, label='Phase W')
            ax.axvline(x=41.55, color=C_MAX, linestyle=':', alpha=0.6, label='Max 41.55A')
            ax.set_title(title, color='white', fontsize=12, fontweight='bold');
            ax.set_xlabel("Phase Current (A)", color='white');
            ax.set_ylabel(ylabel, color='white')
            ax.tick_params(colors='white');
            ax.grid(True, alpha=0.15, color='white');
            ax.legend(facecolor='#222222', edgecolor='#555555')
        self.fig.suptitle("Total Path Resistance: U vs V vs W", color='white', fontsize=16, fontweight='bold', y=0.98)
        self.canvas.draw()


# ==============================================================================
# TAB 6: ADC SHIELDING
# ==============================================================================
class ShieldingTab(QWidget):
    def __init__(self, geom):
        super().__init__()
        self.geom = geom;
        self.layout = QVBoxLayout(self)
        plt.style.use('dark_background')
        self.fig = Figure(figsize=(16, 10), facecolor='#111111')
        self.fig.subplots_adjust(left=0.05, right=0.95, top=0.92, bottom=0.05)
        self.canvas = FigureCanvas(self.fig);
        self.layout.addWidget(self.canvas)
        self.analyze()

    def analyze(self):
        tracks = self.geom.get("tracks", []);
        pours = self.geom.get("pours", [])
        o = self.geom["outline"]
        bx, by, bw, bh = o["x_mm"], o["y_mm"], o["w_mm"], o["h_mm"]
        aggressor_items = []
        for t in tracks:
            if is_power_path(t.get("net_name")): aggressor_items.append(
                {"type": "track", "layer": t["layer"], "data": t})
        for p in pours:
            if not p.get("is_hole") and is_power_path(p.get("net_name")): aggressor_items.append(
                {"type": "pour", "layer": p["layer"], "data": p})
        victim_tracks = [t for t in tracks if "ADC" in (t.get("net_name") or "").upper()]
        ax = self.fig.add_subplot(111);
        ax.set_facecolor('#111111');
        ax.set_aspect('equal');
        ax.axis('off')
        ax.set_xlim(bx - 5, bx + bw + 5);
        ax.set_ylim(by + bh + 5, by - 5)
        ax.add_patch(plt.Rectangle((bx, by), bw, bh, fill=False, edgecolor='#333333', linewidth=0.5, linestyle='--'))
        for a in aggressor_items:
            if a["type"] == "track":
                ax.plot([a["data"]["x1"], a["data"]["x2"]], [a["data"]["y1"], a["data"]["y2"]], color='#ff0055',
                        alpha=0.3, linewidth=2)
            else:
                ax.add_patch(
                    Polygon(a["data"]["points"], closed=True, facecolor='#ff0055', alpha=0.15, edgecolor='none'))
        for v in victim_tracks:
            ax.plot([v["x1"], v["x2"]], [v["y1"], v["y2"]], color='#00f0ff', linewidth=3, solid_capstyle='round')
        ax.set_title("ADC Trace Shielding (Coplanar & Broadside)", color='white', fontsize=16, fontweight='bold')
        self.canvas.draw()


# ==============================================================================
# TAB 7: TENSOR VALIDATION
# ==============================================================================
class TensorValidationTab(QWidget):
    def __init__(self, geom):
        super().__init__()
        self.geom = geom;
        self.layout = QVBoxLayout(self)
        plt.style.use('dark_background')
        self.fig = Figure(figsize=(16, 10), facecolor='#111111')
        self.fig.subplots_adjust(left=0.05, right=0.92, top=0.92, bottom=0.05)
        self.canvas = FigureCanvas(self.fig);
        self.layout.addWidget(self.canvas)
        self.visualize()

    def visualize(self):
        o = self.geom["outline"]
        bx, by, bw, bh = o["x_mm"], o["y_mm"], o["w_mm"], o["h_mm"]
        nx = int(math.ceil(bw / GRID_DX_MM));
        ny = int(math.ceil(bh / GRID_DX_MM));
        nz = 6
        tensor = [[[{"copper": 0.0, "vias": 0} for x in range(nx)] for y in range(ny)] for z in range(nz)]
        for p in self.geom.get("pours", []):
            layer = p.get("layer")
            if layer not in LAYERS or "GND" not in p.get("net_name", "").upper(): continue
            z = LAYERS.index(layer);
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
                    if inside[i_local, j_local]: tensor[z][i_global][j_global]["copper"] = 1.0
        for t in self.geom.get("tracks", []):
            layer = t.get("layer")
            if layer not in LAYERS or "GND" not in t.get("net_name", "").upper(): continue
            z = LAYERS.index(layer);
            x1, y1, x2, y2 = t["x1"], t["y1"], t["x2"], t["y2"]
            length = math.hypot(x2 - x1, y2 - y1);
            steps = max(2, int(length / (GRID_DX_MM / 2)))
            for s in range(steps + 1):
                f = s / steps;
                px = x1 + (x2 - x1) * f;
                py = y1 + (y2 - y1) * f
                j = int((px - bx) / GRID_DX_MM);
                i = int((py - by) / GRID_DX_MM)
                if 0 <= i < ny and 0 <= j < nx: tensor[z][i][j]["copper"] = 1.0
        for v in self.geom.get("vias", []):
            j = int((v["x_mm"] - bx) / GRID_DX_MM);
            i = int((v["y_mm"] - by) / GRID_DX_MM)
            if 0 <= i < ny and 0 <= j < nx:
                for z in range(nz): tensor[z][i][j]["vias"] += 1
        for z in range(nz):
            ax = self.fig.add_subplot(2, 3, z + 1)
            ax.set_facecolor('#111111');
            ax.set_aspect('equal');
            ax.axis('off')
            ax.set_xlim(bx - 2, bx + bw + 2);
            ax.set_ylim(by + bh + 2, by - 2)
            ax.set_title(f"{LAYERS[z]}", color='white', fontweight='bold')
            copper_grid = np.zeros((ny, nx))
            for i in range(ny):
                for j in range(nx):
                    if tensor[z][i][j]["copper"] > 0.5: copper_grid[i, j] = 1
            ax.imshow(copper_grid, extent=[bx, bx + nx * GRID_DX_MM, by + ny * GRID_DX_MM, by], origin='upper',
                      cmap='copper', alpha=0.5, interpolation='nearest')
            for i in range(ny):
                for j in range(nx):
                    if tensor[z][i][j]["vias"] > 0: ax.plot(bx + (j + 0.5) * GRID_DX_MM, by + (i + 0.5) * GRID_DX_MM,
                                                            '.', color='white', markersize=1)
            for fp in self.geom.get("footprints", []):
                ref = fp["ref"].upper()
                if ref != "TP5" and ref not in LOW_SIDE_FETS: continue
                fp_layer = 0 if fp.get("layer") == "F_Cu" else 5
                if fp_layer != z: continue
                color = '#00ff00' if ref == "TP5" else '#ff0055'
                for pad in fp["pads"]:
                    w, h = pad["w_mm"], pad["h_mm"];
                    x, y = pad["x_mm"], pad["y_mm"]
                    angle = pad.get("angle_deg", 0.0)
                    if abs(angle) < 0.01:
                        rect = Rectangle((x - w / 2, y - h / 2), w, h, facecolor=color, edgecolor='white',
                                         linewidth=0.5, alpha=0.8, zorder=10)
                        ax.add_patch(rect)
                    else:
                        rad = math.radians(-angle);
                        ca, sa = math.cos(rad), math.sin(rad);
                        hw, hh = w / 2, h / 2
                        pts = [(x + dx * ca - dy * sa, y + dx * sa + dy * ca) for dx, dy in
                               [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]]
                        poly = Polygon(pts, closed=True, facecolor=color, edgecolor='white', linewidth=0.5, alpha=0.8,
                                       zorder=10)
                        ax.add_patch(poly)
                ax.text(fp["x_mm"], fp["y_mm"], ref, fontsize=6, color='white', ha='center', va='center', weight='bold',
                        zorder=11)
        self.canvas.draw()


# ==============================================================================
# TAB 8: LIVE GROUND BOUNCE SOLVER
# ==============================================================================
class GroundBounceLiveTab(QWidget):
    def __init__(self, geom):
        super().__init__()
        self.geom = geom
        self.layout = QVBoxLayout(self)
        self.ctrl_layout = QHBoxLayout()
        self.lbl_current = QLabel("Phase Current: 0.0 A")
        self.lbl_current.setStyleSheet("font-size: 20px; font-weight: bold; color: #00f0ff;")
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, int(MAX_CURRENT * 10));
        self.slider.setValue(0)
        self.slider.valueChanged.connect(self.update_current)
        self.ctrl_layout.addWidget(self.lbl_current);
        self.ctrl_layout.addWidget(self.slider)
        self.layout.addLayout(self.ctrl_layout)
        plt.style.use('dark_background')
        self.fig = Figure(figsize=(16, 9), facecolor='#111111')
        self.fig.subplots_adjust(left=0.02, right=0.92, top=0.92, bottom=0.05, wspace=0.05, hspace=0.15)
        self.canvas = FigureCanvas(self.fig);
        self.layout.addWidget(self.canvas)
        print("[*] Ground Bounce Tab: Building 3D Grid and Factorizing Matrix...")
        self.setup_solver()
        print("[*] Ground Bounce Tab: Ready.")
        self.update_current()

    def setup_solver(self):
        o = self.geom["outline"]
        self.bx, self.by, self.bw, self.bh = o["x_mm"], o["y_mm"], o["w_mm"], o["h_mm"]
        self.nx = int(math.ceil(self.bw / GRID_DX_MM));
        self.ny = int(math.ceil(self.bh / GRID_DX_MM));
        self.nz = 6
        nx, ny, nz = self.nx, self.ny, self.nz
        bx, by, bw, bh = self.bx, self.by, self.bw, self.bh

        self.tensor = [[[{"copper": 0.0, "vias": 0, "tags": []} for x in range(nx)] for y in range(ny)] for z in
                       range(nz)]
        self.solder_mask = np.zeros((nz, ny, nx), dtype=bool)  # Initialized safely here

        for p in self.geom.get("pours", []):
            layer = p.get("layer")
            if layer not in LAYERS or "GND" not in p.get("net_name", "").upper(): continue
            z = LAYERS.index(layer);
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
                    if inside[i_local, j_local]: self.tensor[z][i_global][j_global]["copper"] = 1.0

        for t in self.geom.get("tracks", []):
            layer = t.get("layer")
            if layer not in LAYERS or "GND" not in t.get("net_name", "").upper(): continue
            z = LAYERS.index(layer);
            x1, y1, x2, y2 = t["x1"], t["y1"], t["x2"], t["y2"]
            length = math.hypot(x2 - x1, y2 - y1);
            steps = max(2, int(length / (GRID_DX_MM / 2)))
            for s in range(steps + 1):
                f = s / steps;
                px = x1 + (x2 - x1) * f;
                py = y1 + (y2 - y1) * f
                j = int((px - bx) / GRID_DX_MM);
                i = int((py - by) / GRID_DX_MM)
                if 0 <= i < ny and 0 <= j < nx: self.tensor[z][i][j]["copper"] = 1.0

        for v in self.geom.get("vias", []):
            j = int((v["x_mm"] - bx) / GRID_DX_MM);
            i = int((v["y_mm"] - by) / GRID_DX_MM)
            if 0 <= i < ny and 0 <= j < nx:
                for z in range(nz): self.tensor[z][i][j]["vias"] += 1

        # Rasterize Solder Mask Openings (Solder Highways)
        for m in self.geom.get("mask_openings", []):
            layer = m.get("layer")
            if layer == "F_Mask":
                z_target = 0
            elif layer == "B_Mask":
                z_target = 5
            else:
                continue

            path = Path(m["points"])
            xs = [pt[0] for pt in m["points"]];
            ys = [pt[1] for pt in m["points"]]
            j0 = max(0, int((min(xs) - bx) / GRID_DX_MM));
            j1 = min(nx - 1, int((max(xs) - bx) / GRID_DX_MM))
            i0 = max(0, int((min(ys) - by) / GRID_DX_MM));
            i1 = min(ny - 1, int((max(ys) - by) / GRID_DX_MM))
            if j1 < j0 or i1 < i0: continue

            xx, yy = np.meshgrid(bx + (np.arange(j0, j1 + 1) + 0.5) * GRID_DX_MM,
                                 by + (np.arange(i0, i1 + 1) + 0.5) * GRID_DX_MM)
            inside = path.contains_points(np.column_stack((xx.flatten(), yy.flatten()))).reshape(
                (i1 - i0 + 1, j1 - j0 + 1))

            for i_local, i_global in enumerate(range(i0, i1 + 1)):
                for j_local, j_global in enumerate(range(j0, j1 + 1)):
                    if inside[i_local, j_local]: self.solder_mask[z_target, i_global, j_global] = True

        def tag_pad_area(fp, ref, z_pad):
            for pad in fp["pads"]:
                w, h = pad["w_mm"], pad["h_mm"];
                px, py = pad["x_mm"], pad["y_mm"]
                j0 = max(0, int((px - w / 2 - bx) / GRID_DX_MM));
                j1 = min(nx - 1, int((px + w / 2 - bx) / GRID_DX_MM))
                i0 = max(0, int((py - h / 2 - by) / GRID_DX_MM));
                i1 = min(ny - 1, int((py + h / 2 - by) / GRID_DX_MM))
                for i in range(i0, i1 + 1):
                    for j in range(j0, j1 + 1):
                        if self.tensor[z_pad][i][j]["copper"] > 0.5: self.tensor[z_pad][i][j]["tags"].append(ref)

        for fp in self.geom.get("footprints", []):
            ref = fp["ref"].upper()
            if not fp.get("pads"): continue
            z_pad = 0 if fp.get("layer") == "F_Cu" else 5
            if ref == "TP5":
                tag_pad_area(fp, "TP5", z_pad)
            elif ref in LOW_SIDE_FETS:
                tag_pad_area(fp, f"FET_{ref}", z_pad)

        N = nz * ny * nx
        rows, cols, vals = [], [], []
        R_sq = COPPER_RESISTIVITY / COPPER_THICKNESS_M;
        G_adj = 1.0 / R_sq;
        G_via = 1000.0

        def k(z, i, j):
            return z * (ny * nx) + i * nx + j

        self.tp5_nodes = [];
        self.fet_nodes = []

        for z in range(nz):
            for i in range(ny):
                for j in range(nx):
                    cell = self.tensor[z][i][j]
                    if "TP5" in cell["tags"]: self.tp5_nodes.append(k(z, i, j))
                    if any(t.startswith("FET_") for t in cell["tags"]): self.fet_nodes.append(k(z, i, j))

                    kk = k(z, i, j);
                    diag = 0.0
                    if cell["copper"] > 0.1:
                        local_G_adj = G_adj * 2.5 if self.solder_mask[z, i, j] else G_adj

                        if j > 0 and self.tensor[z][i][j - 1]["copper"] > 0.1:
                            bridge_G = local_G_adj * 2.5 if self.solder_mask[z, i, j - 1] else local_G_adj
                            rows.append(kk);
                            cols.append(k(z, i, j - 1));
                            vals.append(-bridge_G);
                            diag += bridge_G
                        if j < nx - 1 and self.tensor[z][i][j + 1]["copper"] > 0.1:
                            bridge_G = local_G_adj * 2.5 if self.solder_mask[z, i, j + 1] else local_G_adj
                            rows.append(kk);
                            cols.append(k(z, i, j + 1));
                            vals.append(-bridge_G);
                            diag += bridge_G
                        if i > 0 and self.tensor[z][i - 1][j]["copper"] > 0.1:
                            bridge_G = local_G_adj * 2.5 if self.solder_mask[z, i - 1, j] else local_G_adj
                            rows.append(kk);
                            cols.append(k(z, i - 1, j));
                            vals.append(-bridge_G);
                            diag += bridge_G
                        if i < ny - 1 and self.tensor[z][i + 1][j]["copper"] > 0.1:
                            bridge_G = local_G_adj * 2.5 if self.solder_mask[z, i + 1, j] else local_G_adj
                            rows.append(kk);
                            cols.append(k(z, i + 1, j));
                            vals.append(-bridge_G);
                            diag += bridge_G

                    if cell["vias"] > 0:
                        g_v = G_via * cell["vias"]
                        for z_target in range(nz):
                            if z_target == z: continue
                            if self.tensor[z_target][i][j]["copper"] > 0.1:
                                rows.append(kk);
                                cols.append(k(z_target, i, j));
                                vals.append(-g_v);
                                diag += g_v

                    if cell["copper"] < 0.1 and cell["vias"] == 0:
                        diag += 1e-9
                    else:
                        diag += 1e-6
                    rows.append(kk);
                    cols.append(kk);
                    vals.append(diag)

        self.fet_nodes = list(set(self.fet_nodes));
        self.tp5_nodes = list(set(self.tp5_nodes))
        A = sp.csr_matrix((vals, (rows, cols)), shape=(N, N))
        self.lu = splu(A.tocsc())
        self.N = N

        self.axes = [];
        self.ims = []
        self.V_display = [np.full((ny, nx), np.nan) for _ in range(nz)]
        self.cmap = plt.cm.jet.copy();
        self.cmap.set_bad('#111111')

        for z in range(nz):
            ax = self.fig.add_subplot(2, 3, z + 1)
            ax.set_facecolor('#111111');
            ax.set_aspect('equal');
            ax.axis('off')
            ax.set_xlim(bx - 2, bx + bw + 2);
            ax.set_ylim(by + bh + 2, by - 2)
            ax.set_title(f"{LAYERS[z]}", color='white', fontweight='bold')
            im = ax.imshow(self.V_display[z], extent=[bx, bx + nx * GRID_DX_MM, by + ny * GRID_DX_MM, by],
                           origin='upper', cmap=self.cmap, alpha=1.0, interpolation='nearest', vmin=0, vmax=50)
            self.axes.append(ax);
            self.ims.append(im)

            for fp in self.geom.get("footprints", []):
                ref = fp["ref"].upper()
                if ref != "TP5" and ref not in LOW_SIDE_FETS: continue
                fp_layer = 0 if fp.get("layer") == "F_Cu" else 5
                if fp_layer != z: continue
                edge_color = '#00ff00' if ref == "TP5" else 'white'
                for pad in fp["pads"]:
                    w, h = pad["w_mm"], pad["h_mm"];
                    x, y = pad["x_mm"], pad["y_mm"]
                    angle = pad.get("angle_deg", 0.0)
                    if abs(angle) < 0.01:
                        rect = Rectangle((x - w / 2, y - h / 2), w, h, facecolor='none', edgecolor=edge_color,
                                         linewidth=1.0, alpha=0.8, zorder=10)
                        ax.add_patch(rect)
                    else:
                        rad = math.radians(-angle);
                        ca, sa = math.cos(rad), math.sin(rad);
                        hw, hh = w / 2, h / 2
                        pts = [(x + dx * ca - dy * sa, y + dx * sa + dy * ca) for dx, dy in
                               [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]]
                        poly = Polygon(pts, closed=True, facecolor='none', edgecolor=edge_color, linewidth=1.0,
                                       alpha=0.8, zorder=10)
                        ax.add_patch(poly)

        self.cbar = self.fig.colorbar(self.ims[0], ax=self.axes, fraction=0.02, pad=0.02)
        self.cbar.set_label("Max Ground Bounce (mV)", color="white", fontsize=14)
        self.cbar.ax.yaxis.set_tick_params(color="white")
        plt.setp(self.cbar.ax.get_yticklabels(), color="white")

        self.mask = np.zeros((nz, ny, nx), dtype=bool)
        for z in range(nz):
            for i in range(ny):
                for j in range(nx):
                    cell = self.tensor[z][i][j]
                    if cell["copper"] < 0.1 and cell["vias"] == 0: self.mask[z, i, j] = True

    def update_current(self):
        current_a = self.slider.value() / 10.0
        self.lbl_current.setText(f"Phase Current: {current_a:.1f} A")
        b = np.zeros(self.N)
        i_per_fet = current_a / len(self.fet_nodes) if self.fet_nodes else 0
        for node in self.fet_nodes: b[node] += i_per_fet
        i_per_tp5 = current_a / len(self.tp5_nodes) if self.tp5_nodes else 0
        for node in self.tp5_nodes: b[node] -= i_per_tp5
        V = self.lu.solve(b)
        v_ref = np.mean([V[n] for n in self.tp5_nodes]) if self.tp5_nodes else 0
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


# ==============================================================================
# MAIN WINDOW
# ==============================================================================
class MainWindow(QMainWindow):
    def __init__(self, geom, netlist, loss_data):
        super().__init__()
        self.setWindowTitle("ESC Digital Twin — Unified Dashboard")
        self.resize(1600, 950)
        self.setStyleSheet(STYLESHEET)
        initial_powers = {"U3": 0.183, "U5": 0.158, "U9": 0.107, "U2": 0.165, "U4": 0.020, "U1": 0.020, "U6": 0.150,
                          "R22": 0.0, "R23": 0.0}
        for i in range(1, 13): initial_powers[f"Q{i}"] = 0.0
        solver = ThermalSolver(geom, initial_powers, h_conv=15.0, h_sink=150.0)
        T_init = solver.solve()
        abs_t_min = min(T.min() for T in T_init)
        abs_t_max = 95.0
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        self.tabs.addTab(ThermalMapTab(geom, solver, abs_t_min, abs_t_max), "Thermal Map")
        self.tabs.addTab(TransientThermalTab(geom), "Transient Thermal")
        self.tabs.addTab(CopperCoverageTab(geom, solver), "Copper Coverage")
        self.tabs.addTab(PolygonMapsTab(geom), "Polygon Maps")
        self.tabs.addTab(PowerLossTab(loss_data), "Power Loss Dashboard")
        self.tabs.addTab(SignalIntegrityTab(geom, netlist), "Signal Integrity")
        self.tabs.addTab(ShieldingTab(geom), "ADC Shielding")
        self.tabs.addTab(TensorValidationTab(geom), "Tensor Validation")
        self.tabs.addTab(GroundBounceLiveTab(geom), "Ground Bounce Live")

    def showEvent(self, event):
        super().showEvent(event)
        self._apply_dark_title_bar()

    def _apply_dark_title_bar(self):
        try:
            import ctypes
            hwnd = int(self.winId())
            for attr in (20, 19):
                v = ctypes.c_int(1)
                res = ctypes.windll.dwmapi.DwmSetWindowAttribute(hwnd, attr, ctypes.byref(v), ctypes.sizeof(v))
                if res == 0: break
        except Exception:
            pass


if __name__ == "__main__":
    try:
        with open(GEOMETRY_JSON) as f:
            geom = json.load(f)
        with open(NETLIST_JSON) as f:
            netlist = json.load(f)
    except FileNotFoundError:
        sys.exit(1)
    r_fet_pair = RDS_ON_PER_FET / FETS_PARALLEL
    r_batt = get_net_resistance(geom, "BATT")
    r_u, r_v, r_w = get_net_resistance(geom, "U"), get_net_resistance(geom, "V"), get_net_resistance(geom, "W")
    r_tu, r_tv, r_tw = get_net_resistance(geom, "TU"), get_net_resistance(geom, "TV"), get_net_resistance(geom, "TW")
    loss_data = {
        "U": {"total": r_batt + r_fet_pair + r_u + R_SHUNT + r_tu, "batt": r_batt, "fet": r_fet_pair, "phase": r_u,
              "shunt": R_SHUNT, "motor": r_tu},
        "V": {"total": r_batt + r_fet_pair + r_v + R_SHUNT + r_tv, "batt": r_batt, "fet": r_fet_pair, "phase": r_v,
              "shunt": R_SHUNT, "motor": r_tv},
        "W": {"total": r_batt + r_fet_pair + r_w + R_SHUNT + r_tw, "batt": r_batt, "fet": r_fet_pair, "phase": r_w,
              "shunt": R_SHUNT, "motor": r_tw}
    }
    app = QApplication(sys.argv)
    win = MainWindow(geom, netlist, loss_data)
    win.show()
    sys.exit(app.exec())