import sys
import json
import math
import numpy as np
import matplotlib

matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout

# --- CONSTANTS ---
GEOMETRY_JSON = "pcb_geometry.json"
COPPER_RESISTIVITY = 1.72e-8
COPPER_THICKNESS_M = 35e-6
RDS_ON_PER_FET = 0.008
FETS_PARALLEL = 2
R_SHUNT = 0.002

# --- NEW COLOR PALETTE ---
C_U = '#00f0ff'  # Neon Cyan
C_V = '#ff0055'  # Crimson/Magenta
C_W = '#ffcc00'  # Amber/Gold
C_BG = '#0a0a0a'  # Slightly darker background
C_MAX = '#ffffff'


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


class LossDashboard(QMainWindow):
    def __init__(self, data):
        super().__init__()
        self.setWindowTitle("ESC Power Loss - All Phases")
        self.resize(1600, 800)
        self.data = data

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        plt.style.use('dark_background')
        self.fig = Figure(figsize=(16, 8), facecolor=C_BG)
        self.fig.subplots_adjust(left=0.06, right=0.95, top=0.90, bottom=0.10, wspace=0.25)
        self.canvas = FigureCanvas(self.fig)
        self.layout.addWidget(self.canvas)

        self.plot_data()

    def plot_data(self):
        ax1 = self.fig.add_subplot(131)

        components = ['Battery', 'FETs', 'Phase', 'Shunt', 'Motor']
        x = np.arange(len(components))
        width = 0.25

        u_vals = [self.data["U"]["batt"] * 1000, self.data["U"]["fet"] * 1000, self.data["U"]["phase"] * 1000,
                  self.data["U"]["shunt"] * 1000, self.data["U"]["motor"] * 1000]
        v_vals = [self.data["V"]["batt"] * 1000, self.data["V"]["fet"] * 1000, self.data["V"]["phase"] * 1000,
                  self.data["V"]["shunt"] * 1000, self.data["V"]["motor"] * 1000]
        w_vals = [self.data["W"]["batt"] * 1000, self.data["W"]["fet"] * 1000, self.data["W"]["phase"] * 1000,
                  self.data["W"]["shunt"] * 1000, self.data["W"]["motor"] * 1000]

        # Draw bars with new colors and alpha transparency
        ax1.bar(x - width, u_vals, width, label='Phase U', color=C_U, alpha=0.85, edgecolor='none')
        ax1.bar(x, v_vals, width, label='Phase V', color=C_V, alpha=0.85, edgecolor='none')
        ax1.bar(x + width, w_vals, width, label='Phase W', color=C_W, alpha=0.85, edgecolor='none')

        ax1.set_title("Resistance Breakdown (mΩ)", color='white', fontsize=12, fontweight='bold')
        ax1.set_ylabel("Resistance (mΩ)", color='white')
        ax1.set_xticks(x)
        ax1.set_xticklabels(components, color='white', fontsize=10)
        ax1.tick_params(axis='y', colors='white')
        ax1.grid(axis='y', alpha=0.15, color='white')
        ax1.legend(facecolor='#222222', edgecolor='#555555')

        # --- PANEL 2 & 3 ---
        ax2 = self.fig.add_subplot(132)
        ax3 = self.fig.add_subplot(133)

        currents = np.linspace(0, 60, 100)

        v_drop_u = currents * (self.data["U"]["total"] * 1000)
        v_drop_v = currents * (self.data["V"]["total"] * 1000)
        v_drop_w = currents * (self.data["W"]["total"] * 1000)

        p_loss_u = (currents ** 2) * self.data["U"]["total"]
        p_loss_v = (currents ** 2) * self.data["V"]["total"]
        p_loss_w = (currents ** 2) * self.data["W"]["total"]

        # Draw lines with new colors, thicker lines
        ax2.plot(currents, v_drop_u, color=C_U, linewidth=2.5, label='Phase U')
        ax2.plot(currents, v_drop_v, color=C_V, linewidth=2.5, label='Phase V')
        ax2.plot(currents, v_drop_w, color=C_W, linewidth=2.5, label='Phase W')
        ax2.axvline(x=41.55, color=C_MAX, linestyle=':', alpha=0.6, label='Max 41.55A')

        ax2.set_title("Voltage Drop vs Current", color='white', fontsize=12, fontweight='bold')
        ax2.set_xlabel("Phase Current (A)", color='white')
        ax2.set_ylabel("Voltage Drop (mV)", color='white')
        ax2.tick_params(colors='white')
        ax2.grid(True, alpha=0.15, color='white')
        ax2.legend(facecolor='#222222', edgecolor='#555555')

        ax3.plot(currents, p_loss_u, color=C_U, linewidth=2.5, label='Phase U')
        ax3.plot(currents, p_loss_v, color=C_V, linewidth=2.5, label='Phase V')
        ax3.plot(currents, p_loss_w, color=C_W, linewidth=2.5, label='Phase W')
        ax3.axvline(x=41.55, color=C_MAX, linestyle=':', alpha=0.6, label='Max 41.55A')

        ax3.set_title("Power Loss vs Current", color='white', fontsize=12, fontweight='bold')
        ax3.set_xlabel("Phase Current (A)", color='white')
        ax3.set_ylabel("Power Dissipated (W)", color='white')
        ax3.tick_params(colors='white')
        ax3.grid(True, alpha=0.15, color='white')
        ax3.legend(facecolor='#222222', edgecolor='#555555')

        self.fig.suptitle("Total Path Resistance: U vs V vs W", color='white', fontsize=16, fontweight='bold', y=0.98)
        self.canvas.draw()


def main():
    try:
        with open(GEOMETRY_JSON) as f:
            geom = json.load(f)
    except FileNotFoundError:
        print("[!] pcb_geometry.json not found.")
        sys.exit(1)

    r_fet_pair = RDS_ON_PER_FET / FETS_PARALLEL

    r_batt = get_net_resistance(geom, "BATT")
    r_u = get_net_resistance(geom, "U")
    r_v = get_net_resistance(geom, "V")
    r_w = get_net_resistance(geom, "W")
    r_tu = get_net_resistance(geom, "TU")
    r_tv = get_net_resistance(geom, "TV")
    r_tw = get_net_resistance(geom, "TW")

    data = {
        "U": {"total": r_batt + r_fet_pair + r_u + R_SHUNT + r_tu, "batt": r_batt, "fet": r_fet_pair, "phase": r_u,
              "shunt": R_SHUNT, "motor": r_tu},
        "V": {"total": r_batt + r_fet_pair + r_v + R_SHUNT + r_tv, "batt": r_batt, "fet": r_fet_pair, "phase": r_v,
              "shunt": R_SHUNT, "motor": r_tv},
        "W": {"total": r_batt + r_fet_pair + r_w + R_SHUNT + r_tw, "batt": r_batt, "fet": r_fet_pair, "phase": r_w,
              "shunt": R_SHUNT, "motor": r_tw}
    }

    print("=" * 60)
    print("  ESC PATH RESISTANCE & VOLTAGE DROP ANALYSIS")
    print("=" * 60)
    print(f"  Battery (+BATT) : {r_batt * 1000:8.3f} mΩ")
    print(f"  FET Switch Pair : {r_fet_pair * 1000:8.3f} mΩ")
    print(f"  Shunt (2mΩ)     : {R_SHUNT * 1000:8.3f} mΩ")
    print("-" * 60)
    print(f"  Phase U Routing : {r_u * 1000:8.3f} mΩ + {r_tu * 1000:8.3f} mΩ (Tu)")
    print(f"  Phase V Routing : {r_v * 1000:8.3f} mΩ + {r_tv * 1000:8.3f} mΩ (Tv)")
    print(f"  Phase W Routing : {r_w * 1000:8.3f} mΩ + {r_tw * 1000:8.3f} mΩ (Tw)")
    print("-" * 60)
    print(f"  TOTAL PHASE U   : {data['U']['total'] * 1000:8.3f} mΩ")
    print(f"  TOTAL PHASE V   : {data['V']['total'] * 1000:8.3f} mΩ")
    print(f"  TOTAL PHASE W   : {data['W']['total'] * 1000:8.3f} mΩ")
    print("=" * 60)

    app = QApplication(sys.argv)
    win = LossDashboard(data)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()