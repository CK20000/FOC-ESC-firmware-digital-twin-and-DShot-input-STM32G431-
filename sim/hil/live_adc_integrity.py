import sys
import numpy as np
import matplotlib

matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from scipy.signal import lsim, StateSpace

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QSlider, QLabel
)
from PyQt6.QtCore import Qt

# --- BOARD CONSTANTS ---
COPPER_RESISTIVITY = 1.72e-8
COPPER_THICKNESS_M = 35e-6
LEN_ADC1 = 69.0  # mm
W_TRACE = 0.3  # mm
R_FILTER = 100.0  # Ohms
C_FILTER = 2.2e-9  # Farads
C_MCU_PIN = 5e-12  # Farads
R_MCU_LEAK = 10e6  # Ohms

# --- MOTOR CONSTANTS ---
PWM_FREQ = 20000.0  # 20 kHz
PWM_PERIOD = 1.0 / PWM_FREQ
MAX_CURRENT = 41.55
MAX_ELEC_FREQ = 500.0  # Hz

# --- ADC CONSTANTS ---
V_REF = 3.3
ADC_BITS = 12
ADC_LSB = V_REF / (2 ** ADC_BITS)  # ~0.8mV


class LiveADCApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Live FOC ADC Signal Integrity")
        self.resize(1600, 900)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        # --- Controls ---
        self.ctrl_layout = QHBoxLayout()

        self.slider_amp = QSlider(Qt.Orientation.Horizontal)
        self.slider_amp.setRange(0, int(MAX_CURRENT * 10))
        self.slider_amp.setValue(210)  # 21A default
        self.slider_amp.valueChanged.connect(self.update_sim)
        self.lbl_amp = QLabel("Current Amplitude: 21.0 A")
        self.lbl_amp.setStyleSheet("font-size: 18px; font-weight: bold; color: #00f0ff;")

        self.slider_freq = QSlider(Qt.Orientation.Horizontal)
        self.slider_freq.setRange(1, int(MAX_ELEC_FREQ))
        self.slider_freq.setValue(200)  # 200Hz default
        self.slider_freq.valueChanged.connect(self.update_sim)
        self.lbl_freq = QLabel("Elec Freq: 200 Hz")
        self.lbl_freq.setStyleSheet("font-size: 18px; font-weight: bold; color: #ff0055;")

        self.ctrl_layout.addWidget(self.lbl_amp)
        self.ctrl_layout.addWidget(self.slider_amp)
        self.ctrl_layout.addWidget(self.lbl_freq)
        self.ctrl_layout.addWidget(self.slider_freq)
        self.layout.addLayout(self.ctrl_layout)

        # --- Plots ---
        plt.style.use('dark_background')
        self.fig = Figure(figsize=(16, 8), facecolor='#111111')
        self.fig.subplots_adjust(left=0.06, right=0.95, top=0.92, bottom=0.08, wspace=0.2, hspace=0.3)
        self.canvas = FigureCanvas(self.fig)
        self.layout.addWidget(self.canvas)

        self.ax1 = self.fig.add_subplot(221)  # Zoomed PWM Period
        self.ax2 = self.fig.add_subplot(222)  # Full Sine Wave
        self.ax3 = self.fig.add_subplot(212)  # Error over time

        self.update_sim()

    def build_state_space(self, L_trace, C_trace):
        R_trace = COPPER_RESISTIVITY * (LEN_ADC1 / 1000.0) / ((W_TRACE / 1000.0) * COPPER_THICKNESS_M)
        A = np.array([
            [-R_trace / L_trace, -1 / L_trace, 0],
            [1 / C_trace, -1 / (R_FILTER * C_trace), 1 / (R_FILTER * C_trace)],
            [0, 1 / (R_FILTER * (C_FILTER + C_MCU_PIN)),
             -1 / (R_FILTER * (C_FILTER + C_MCU_PIN)) - 1 / (R_MCU_LEAK * (C_FILTER + C_MCU_PIN))]
        ])
        B = np.array([[1 / L_trace], [0], [0]])
        C = np.array([[0, 0, 1]])
        D = np.array([[0]])
        return StateSpace(A, B, C, D)

    def update_sim(self):
        I_amp = self.slider_amp.value() / 10.0
        f_elec = self.slider_freq.value()

        self.lbl_amp.setText(f"Current Amplitude: {I_amp:.1f} A")
        self.lbl_freq.setText(f"Elec Freq: {f_elec} Hz")

        # 1. Zoom Simulation (1 PWM period)
        t_zoom = np.linspace(0, PWM_PERIOD, 5000)
        # Baseline voltage at this specific current
        v_base_zoom = 1.65 + (I_amp * 0.002 * 20)

        v_ideal_zoom = np.full_like(t_zoom, v_base_zoom)

        # Inject 2V noise spike at t=0
        v_noisy_zoom = np.copy(v_ideal_zoom)
        spike_mask = (t_zoom >= 0) & (t_zoom < 20e-9)
        v_noisy_zoom[spike_mask] += 2.0

        L1 = LEN_ADC1 * 1e-9
        C1 = LEN_ADC1 * 1e-12
        sys1 = self.build_state_space(L1, C1)
        _, y_meas_zoom, _ = lsim(sys1, U=v_noisy_zoom, T=t_zoom, X0=[0, v_base_zoom, v_base_zoom])

        # 2. Macro Simulation (Sine wave)
        t_macro = np.linspace(0, 2.0 / f_elec, 5000)
        I_phase = I_amp * np.sin(2 * np.pi * f_elec * t_macro)

        # 3. Realistic Sampling
        sample_times = np.arange(0, t_macro[-1], PWM_PERIOD)
        sample_times = sample_times[sample_times < t_macro[-1]]

        I_real_sampled = I_amp * np.sin(2 * np.pi * f_elec * sample_times)
        v_real_sampled = 1.65 + (I_real_sampled * 0.002 * 20)

        # Add tiny residual noise (5mV)
        v_noisy_sampled = v_real_sampled + np.random.normal(0, 0.005, len(sample_times))

        # Quantize to 12-bit ADC
        v_quantized = np.round(v_noisy_sampled / ADC_LSB) * ADC_LSB
        I_measured_sampled = (v_quantized - 1.65) / (0.002 * 20)

        error_A = I_measured_sampled - I_real_sampled

        # --- Plotting ---
        self.ax1.clear()
        self.ax1.plot(t_zoom * 1e6, v_ideal_zoom, color='#888888', linestyle='--', linewidth=2, label='Ideal AD8418')
        self.ax1.plot(t_zoom * 1e6, v_noisy_zoom, color='#ffcc00', alpha=0.5, linewidth=2, label='Noisy Signal')
        self.ax1.plot(t_zoom * 1e6, y_meas_zoom, color='#00f0ff', linewidth=3, label='ADC Pin (Filtered)')
        self.ax1.axvline(x=(PWM_PERIOD / 2) * 1e6, color='#ff0055', linestyle=':', linewidth=2, label='MCU Samples')

        # ZOOM IN THE Y-AXIS: Center on the filtered settling curve
        self.ax1.set_ylim(v_base_zoom - 0.03, v_base_zoom + 0.15)

        self.ax1.set_title("Zoom: 1 PWM Period (50us) - Y-Axis Zoomed to Settling", color='white', fontweight='bold')
        self.ax1.set_xlabel("Time (us)", color='white')
        self.ax1.set_ylabel("Voltage (V)", color='white')
        self.ax1.tick_params(colors='white')
        self.ax1.grid(True, alpha=0.2)
        self.ax1.legend(loc='upper right')

        self.ax2.clear()
        self.ax2.plot(t_macro * 1e3, I_phase, color='#00f0ff', alpha=0.8, linewidth=2, label='Real Phase Current')
        self.ax2.scatter(sample_times * 1e3, I_measured_sampled, color='#ff0055', s=20, zorder=5,
                         label='Sampled Current (12-bit)')
        self.ax2.set_title("Real Current vs Sampled Current (Full Sine Wave)", color='white', fontweight='bold')
        self.ax2.set_xlabel("Time (ms)", color='white')
        self.ax2.set_ylabel("Current (A)", color='white')
        self.ax2.tick_params(colors='white')
        self.ax2.grid(True, alpha=0.2)
        self.ax2.legend(loc='upper right')

        self.ax3.clear()
        self.ax3.plot(sample_times * 1e3, error_A, color='#ffcc00', linewidth=2, marker='o', markersize=3)
        self.ax3.axhline(0, color='#888888', linestyle='--')
        self.ax3.set_title("Current Measurement Error (ADC Quantization + Residual Noise)", color='white',
                           fontweight='bold')
        self.ax3.set_xlabel("Time (ms)", color='white')
        self.ax3.set_ylabel("Error (A)", color='white')
        self.ax3.tick_params(colors='white')
        self.ax3.grid(True, alpha=0.2)

        max_err = max(np.max(np.abs(error_A)), 0.05)
        self.ax3.set_ylim(-max_err * 1.2, max_err * 1.2)

        self.canvas.draw()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = LiveADCApp()
    win.show()
    sys.exit(app.exec())