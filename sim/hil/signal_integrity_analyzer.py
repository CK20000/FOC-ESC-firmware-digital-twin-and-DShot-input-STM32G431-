import sys
import json
import math
import numpy as np
import matplotlib

matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import re

from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout

GEOMETRY_JSON = "pcb_geometry.json"
NETLIST_JSON = "digital_twin_netlist.json"
COPPER_RESISTIVITY = 1.72e-8
COPPER_THICKNESS_M = 35e-6

# Gate Drive Electrical Parameters
R_GATE = 10.0  # 10Ω gate resistor
R_DRIVER = 2.0  # EG2134 internal impedance
L_PER_MM = 1.0  # 1 nH per mm of trace


def get_total_length(geom, net_name):
    total_length_mm = 0.0
    for t in geom.get("tracks", []):
        if (t.get("net_name") or "").upper() == net_name.upper():
            dx = t["x2"] - t["x1"]
            dy = t["y2"] - t["y1"]
            total_length_mm += math.hypot(dx, dy)
    return total_length_mm


def extract_rc_filter(netlist, start_net):
    r_val, c_val = None, None
    r_ref = None
    if start_net in netlist:
        for node in netlist[start_net]["nodes"]:
            if node["component"].startswith("R"):
                r_ref = node["component"]
                try:
                    r_val = float(re.sub(r'[^0-9.]', '', node["value"]))
                except:
                    pass
                break
    if not r_ref: return 100.0, 2.2e-9

    other_net = None
    for net_name, data in netlist.items():
        if net_name == start_net: continue
        for node in data["nodes"]:
            if node["component"] == r_ref:
                other_net = net_name
                break
        if other_net: break

    if not other_net: return (r_val or 100.0), 2.2e-9

    if other_net in netlist:
        for node in netlist[other_net]["nodes"]:
            if node["component"].startswith("C"):
                try:
                    if "n" in node["value"].lower():
                        c_val = float(re.sub(r'[^0-9.]', '', node["value"])) * 1e-9
                    elif "u" in node["value"].lower():
                        c_val = float(re.sub(r'[^0-9.]', '', node["value"])) * 1e-6
                    elif "p" in node["value"].lower():
                        c_val = float(re.sub(r'[^0-9.]', '', node["value"])) * 1e-12
                    else:
                        c_val = float(re.sub(r'[^0-9.]', '', node["value"])) * 1e-12
                except:
                    pass
                break
    return (r_val or 100.0), (c_val or 2.2e-9)


class SignalIntegrityApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Signal Integrity & FET Switching Analysis")
        self.resize(1600, 1000)

        with open(GEOMETRY_JSON) as f:
            geom = json.load(f)
        try:
            with open(NETLIST_JSON) as f:
                netlist = json.load(f)
        except FileNotFoundError:
            netlist = {}

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        plt.style.use('dark_background')
        self.fig = Figure(figsize=(16, 10), facecolor='#050505')
        self.fig.subplots_adjust(left=0.08, right=0.95, top=0.90, bottom=0.08, wspace=0.25, hspace=0.35)
        self.canvas = FigureCanvas(self.fig)
        self.layout.addWidget(self.canvas)

        self.analyze(geom, netlist)

    def analyze(self, geom, netlist):
        len_adc1 = get_total_length(geom, "ADC1")
        len_adc2 = get_total_length(geom, "ADC2")
        r_adc1, c_adc1 = extract_rc_filter(netlist, "ADC1")
        r_adc2, c_adc2 = extract_rc_filter(netlist, "ADC2")

        w_trace = 0.3
        R1 = COPPER_RESISTIVITY * (len_adc1 / 1000.0) / ((w_trace / 1000.0) * COPPER_THICKNESS_M)
        L1 = len_adc1 * 1e-9
        C1 = len_adc1 * 1e-12

        R2 = COPPER_RESISTIVITY * (len_adc2 / 1000.0) / ((w_trace / 1000.0) * COPPER_THICKNESS_M)
        L2 = len_adc2 * 1e-9
        C2 = len_adc2 * 1e-12

        # Gate Loop Analysis
        driver_nets = ["HO1", "LO1", "HO2", "LO2", "HO3", "LO3"]
        fet_gate_nets = {}
        pattern_q = re.compile(r'^Net-\((Q\d+)-G\)$', re.IGNORECASE)
        for t in geom.get("tracks", []):
            net = t.get("net_name") or ""
            match = pattern_q.match(net)
            if match:
                q_ref = match.group(1).upper()
                if net not in fet_gate_nets:
                    fet_gate_nets[net] = {"ref": q_ref, "length": 0.0}
                dx = t["x2"] - t["x1"]
                dy = t["y2"] - t["y1"]
                fet_gate_nets[net]["length"] += math.hypot(dx, dy)

        switch_map = {
            "HO1": ["Q1", "Q3"], "LO1": ["Q4", "Q2"],
            "HO2": ["Q5", "Q7"], "LO2": ["Q8", "Q6"],
            "HO3": ["Q9", "Q11"], "LO3": ["Q12", "Q10"]
        }

        bar_labels = []
        driver_lengths = []
        fet_lengths = []

        for drv in driver_nets:
            drv_len = get_total_length(geom, drv)
            for q_ref in switch_map.get(drv, []):
                for net, data in fet_gate_nets.items():
                    if data["ref"] == q_ref:
                        bar_labels.append(f"{drv} -> {q_ref}")
                        driver_lengths.append(drv_len)
                        fet_lengths.append(data["length"])
                        break

        # Transient Simulation for ADC
        t = np.linspace(0, 1000e-9, 5000)
        v_src = np.where((t >= 110e-9) & (t < 130e-9), 3.65, 1.65)

        def simulate(t, v_src, R_tr, L_tr, C_tr, R_flt, C_flt):
            v_mcu = np.zeros_like(t)
            v_mcu[0] = 1.65
            i_L = 0.0
            v_trace = 1.65
            v_mcu_node = 1.65
            R_MCU_LEAK = 10e6
            C_MCU_PIN = 5e-12
            for i in range(1, len(t)):
                dt = t[i] - t[i - 1]
                i_filter = (v_trace - v_mcu_node) / R_flt
                di_L = (v_src[i] - v_trace - i_L * R_tr) / L_tr
                dv_trace = (i_L - i_filter) / C_tr
                i_mcu_leak = v_mcu_node / R_MCU_LEAK
                dv_mcu = (i_filter - i_mcu_leak) / (C_flt + C_MCU_PIN)
                i_L += di_L * dt
                v_trace += dv_trace * dt
                v_mcu_node += dv_mcu * dt
                v_mcu[i] = v_mcu_node
            return v_mcu

        v_mcu1 = simulate(t, v_src, R1, L1, C1, r_adc1, c_adc1)
        v_mcu2 = simulate(t, v_src, R2, L2, C2, r_adc2, c_adc2)

        # --- Plotting ---
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

        # Plot 3: Stacked Lengths with Switching Delay text
        ax3 = self.fig.add_subplot(223)
        x = np.arange(len(bar_labels))
        ax3.bar(x, driver_lengths, color='#ffb733', label='Driver Trace', edgecolor='none')
        ax3.bar(x, fet_lengths, bottom=driver_lengths, color='#ff0055', label='FET Gate Trace', edgecolor='none')

        # Add Delay Text on top of bars
        for i in range(len(bar_labels)):
            tot_len = driver_lengths[i] + fet_lengths[i]
            delay_ns = tot_len / (R_GATE + R_DRIVER)  # Since L_per_mm=1, tot_len = tot_nH
            ax3.text(i, tot_len + 2, f"{delay_ns:.1f}ns", ha='center', va='bottom', color='#00f0ff', fontsize=9,
                     fontweight='bold')

        ax3.set_xticks(x)
        ax3.set_xticklabels(bar_labels, rotation=45, ha='right', color='white', fontsize=8)
        ax3.set_title("Total Gate Loop Length & Switching Delay", color='white', fontweight='bold')
        ax3.set_ylabel("Length (mm)", color='white')
        ax3.set_ylim(0, max(
            driver_lengths[i] + fet_lengths[i] for i in range(len(bar_labels))) * 1.15)  # Make room for text
        ax3.tick_params(colors='white');
        ax3.grid(axis='y', alpha=0.2)
        ax3.legend(facecolor='#222222', edgecolor='#555555')

        # Plot 4: Peak Noise (Restored)
        ax4 = self.fig.add_subplot(224)
        spike_window = (t >= 100e-9) & (t <= 150e-9)
        peak_noise_1 = np.max(np.abs(v_mcu1[spike_window] - 1.65))
        peak_noise_2 = np.max(np.abs(v_mcu2[spike_window] - 1.65))

        bars = ax4.bar(["ADC1 (Phase U)", "ADC2 (Phase W)"],
                       [peak_noise_1, peak_noise_2],
                       color=['#ff0055', '#ffcc00'], alpha=0.8, width=0.4)
        ax4.set_title("Peak Noise Voltage at MCU (After RC Filter)", color='white', fontweight='bold')
        ax4.set_ylabel("Spike Amplitude (V)", color='white')
        ax4.tick_params(colors='white');
        ax4.grid(axis='y', alpha=0.2)

        for bar in bars:
            yval = bar.get_height()
            label = f'{yval * 1000:.1f} mV' if yval < 0.1 else f'{yval:.3f} V'
            ax4.text(bar.get_x() + bar.get_width() / 2, yval + 0.005, label,
                     ha='center', va='bottom', color='white', fontsize=12, fontweight='bold')

        self.fig.suptitle("Signal Integrity & FET Switching Analysis", color='white', fontsize=18, fontweight='bold')
        self.canvas.draw()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = SignalIntegrityApp()
    win.show()
    sys.exit(app.exec())