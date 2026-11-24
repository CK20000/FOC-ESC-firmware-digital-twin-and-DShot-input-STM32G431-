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
    QSlider, QLabel, QPushButton, QCheckBox, QDoubleSpinBox, QFormLayout, QGroupBox
)
from PyQt6.QtCore import Qt, QTimer

# --- CONSTANTS ---
V_BUS = 24.0  # 6S Battery
POLES = 14
LEN_ADC1 = 69.0  # mm
LEN_ADC2 = 61.0  # mm
COPPER_RESISTIVITY = 1.72e-8
COPPER_THICKNESS_M = 35e-6
TRACE_WIDTH_MM = 0.3

R_FILTER = 100.0
C_FILTER = 2.2e-9
V_REF = 3.3
ADC_BITS = 12
ADC_LSB = V_REF / (2 ** ADC_BITS)
R_SHUNT = 0.002
GAIN = 20

# --- FIXED MOTOR CONSTANT ---
# Lowered KV to 520 so back-EMF builds up realistically at 12k RPM
KV_UNLOADED = 520.0


class MotorADCApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Live FOC Physics: Phase U & Phase W")
        self.resize(1700, 900)

        self.is_running = False
        self.time_offset = 0.0
        self.auto_sweep = False

        # Motor inertia state
        self.current_rpm = 0.0

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QHBoxLayout(self.central_widget)

        # ================= LEFT SIDE: PLOTS =================
        self.plot_widget = QWidget()
        self.plot_layout = QVBoxLayout(self.plot_widget)
        self.main_layout.addWidget(self.plot_widget, stretch=4)

        plt.style.use('dark_background')
        self.fig = Figure(figsize=(14, 8), facecolor='#111111')
        self.fig.subplots_adjust(left=0.06, right=0.95, top=0.92, bottom=0.08, wspace=0.2, hspace=0.3)
        self.canvas = FigureCanvas(self.fig)
        self.plot_layout.addWidget(self.canvas)

        self.ax1 = self.fig.add_subplot(221)
        self.ax2 = self.fig.add_subplot(222)
        self.ax3 = self.fig.add_subplot(212)

        self._init_plots()

        # ================= RIGHT SIDE: CONTROLS =================
        self.ctrl_panel = QWidget()
        self.ctrl_panel.setFixedWidth(350)
        self.ctrl_layout = QVBoxLayout(self.ctrl_panel)
        self.ctrl_panel.setStyleSheet("""
            QWidget { background: #0a0a0a; color: white; font-family: Arial; }
            QLabel { font-size: 14px; }
            QGroupBox { border: 1px solid #444; margin-top: 10px; font-weight: bold; color: #00f0ff; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px; }
            QDoubleSpinBox { background: #222; color: #fff; border: 1px solid #555; padding: 2px; }
        """)
        self.main_layout.addWidget(self.ctrl_panel)

        gb_throttle = QGroupBox("Motor & Throttle")
        lay_throttle = QFormLayout()

        self.slider_throttle = QSlider(Qt.Orientation.Horizontal)
        self.slider_throttle.setRange(0, 100)
        self.slider_throttle.setValue(50)
        self.lbl_throttle = QLabel("50%")
        h_throttle = QHBoxLayout()
        h_throttle.addWidget(self.slider_throttle)
        h_throttle.addWidget(self.lbl_throttle)
        lay_throttle.addRow("Throttle:", h_throttle)

        self.spin_rpm_max = QDoubleSpinBox()
        self.spin_rpm_max.setRange(1000, 30000)
        self.spin_rpm_max.setValue(12000)
        lay_throttle.addRow("Max Loaded RPM:", self.spin_rpm_max)

        self.spin_r_motor = QDoubleSpinBox()
        self.spin_r_motor.setRange(1, 500)
        self.spin_r_motor.setValue(50.0)
        lay_throttle.addRow("Motor R (mΩ):", self.spin_r_motor)

        self.spin_l_motor = QDoubleSpinBox()
        self.spin_l_motor.setRange(1, 100)
        self.spin_l_motor.setValue(10.0)
        lay_throttle.addRow("Motor L (μH):", self.spin_l_motor)

        gb_throttle.setLayout(lay_throttle)
        self.ctrl_layout.addWidget(gb_throttle)

        gb_sample = QGroupBox("ADC Sampling")
        lay_sample = QFormLayout()
        self.spin_sample_offset = QDoubleSpinBox()
        self.spin_sample_offset.setRange(0.0, 50.0)
        self.spin_sample_offset.setValue(25.0)
        self.spin_sample_offset.setSingleStep(0.1)
        lay_sample.addRow("Sample Offset (us):", self.spin_sample_offset)
        gb_sample.setLayout(lay_sample)
        self.ctrl_layout.addWidget(gb_sample)

        self.btn_play = QPushButton("▶ Play")
        self.btn_play.setCheckable(True)
        self.btn_play.setStyleSheet("font-size: 18px; padding: 10px; background: #222; border: 1px solid #555;")
        self.btn_play.clicked.connect(self.toggle_play)
        self.ctrl_layout.addWidget(self.btn_play)

        self.cb_sweep = QCheckBox("Auto-Sweep Throttle")
        self.cb_sweep.setStyleSheet("font-size: 16px; margin-top: 10px;")
        self.cb_sweep.stateChanged.connect(self.toggle_sweep)
        self.ctrl_layout.addWidget(self.cb_sweep)

        self.lbl_status = QLabel("")
        self.lbl_status.setStyleSheet("font-size: 14px; color: #ffcc00; margin-top: 20px;")
        self.ctrl_layout.addWidget(self.lbl_status)
        self.ctrl_layout.addStretch()

        self.timer = QTimer()
        self.timer.setInterval(30)
        self.timer.timeout.connect(self.animate)

    def _init_plots(self):
        self.line_raw_u, = self.ax1.plot([], [], color='#ffcc00', alpha=0.3, label='Raw U')
        self.line_raw_w, = self.ax1.plot([], [], color='#ffcc00', alpha=0.3, linestyle='--', label='Raw W')
        self.line_adc1, = self.ax1.plot([], [], color='#00f0ff', linewidth=2.5, label='ADC1 (Phase U)')
        self.line_adc2, = self.ax1.plot([], [], color='#ff0055', linewidth=2.5, linestyle='--', label='ADC2 (Phase W)')
        self.ax1.axvline(x=25, color='white', linestyle=':', label='ADC Sample')
        self.ax1.set_title("Zoom: 1 PWM Period (Instantaneous Voltage)", color='white', fontweight='bold')
        self.ax1.set_xlabel("Time (us)", color='white');
        self.ax1.set_ylabel("Voltage (V)", color='white')
        self.ax1.tick_params(colors='white');
        self.ax1.grid(True, alpha=0.2);
        self.ax1.legend(loc='upper right')
        self.ax1.set_xlim(0, 50);
        self.ax1.set_ylim(-0.1, 3.4)

        self.line_real_u, = self.ax2.plot([], [], color='#00f0ff', linewidth=2, label='Phase U')
        self.line_real_w, = self.ax2.plot([], [], color='#ffcc00', linewidth=2, label='Phase W')
        self.scatter_u = self.ax2.scatter([], [], color='#00f0ff', s=20, zorder=5)
        self.scatter_w = self.ax2.scatter([], [], color='#ffcc00', s=20, zorder=5)
        self.ax2.set_title("Real Motor Currents (120° Phase Shift)", color='white', fontweight='bold')
        self.ax2.set_xlabel("Time (ms)", color='white');
        self.ax2.set_ylabel("Current (A)", color='white')
        self.ax2.tick_params(colors='white');
        self.ax2.grid(True, alpha=0.2);
        self.ax2.legend(loc='upper right')
        self.ax2.set_xlim(0, 2.0)

        self.line_err1, = self.ax3.plot([], [], color='#00f0ff', linewidth=2, label='ADC1 Error (U)')
        self.line_err2, = self.ax3.plot([], [], color='#ff0055', linewidth=2, linestyle='--', label='ADC2 Error (W)')
        self.ax3.axhline(0, color='#888888', linestyle='--')
        self.ax3.set_title("ADC Error (Quantization + ADC Clipping)", color='white', fontweight='bold')
        self.ax3.set_xlabel("Time (ms)", color='white');
        self.ax3.set_ylabel("Error (A)", color='white')
        self.ax3.tick_params(colors='white');
        self.ax3.grid(True, alpha=0.2)
        self.ax3.set_xlim(0, 2.0);
        self.ax3.legend(loc='upper right')

    def build_state_space(self, L_trace, C_trace, R_trace):
        A = np.array([
            [-R_trace / L_trace, -1 / L_trace, 0],
            [1 / C_trace, -1 / (R_FILTER * C_trace), 1 / (R_FILTER * C_trace)],
            [0, 1 / (R_FILTER * (C_FILTER + 5e-12)),
             -1 / (R_FILTER * (C_FILTER + 5e-12)) - 1 / (10e6 * (C_FILTER + 5e-12))]
        ])
        B = np.array([[1 / L_trace], [0], [0]])
        C = np.array([[0, 0, 1]])
        D = np.array([[0]])
        return StateSpace(A, B, C, D)

    def toggle_play(self):
        self.is_running = self.btn_play.isChecked()
        if self.is_running:
            self.btn_play.setText("⏸ Pause")
            self.timer.start()
        else:
            self.btn_play.setText("▶ Play")
            self.timer.stop()

    def toggle_sweep(self):
        self.auto_sweep = self.cb_sweep.isChecked()

    def animate(self):
        self.time_offset += 0.00005

        if self.auto_sweep:
            sweep = (np.sin(self.time_offset * 100) + 1) / 2 * 100
            self.slider_throttle.setValue(int(sweep))

        throttle = self.slider_throttle.value() / 100.0
        max_rpm = self.spin_rpm_max.value()
        sample_us = self.spin_sample_offset.value()

        r_m = self.spin_r_motor.value() / 1000.0
        l_m = self.spin_l_motor.value() * 1e-6

        # 1. Motor Physics (Fixed with Inertia)
        target_rpm = throttle * max_rpm

        # Simulate motor inertia (ramps up to target RPM)
        self.current_rpm += (target_rpm - self.current_rpm) * 0.05

        rpm = self.current_rpm
        omega_e = rpm * (POLES / 2.0) * (2 * np.pi / 60.0)
        f_elec = omega_e / (2 * np.pi)

        v_applied_peak = throttle * (V_BUS / np.sqrt(3))

        # Back-EMF (Using fixed KV so it actually fights the voltage at 12k RPM)
        e_peak = (rpm / KV_UNLOADED) / np.sqrt(3)

        z_mag = np.sqrt(r_m ** 2 + (omega_e * l_m) ** 2)
        z_angle = np.arctan2(omega_e * l_m, r_m)

        delta_v = v_applied_peak - e_peak
        if delta_v < 0: delta_v = 0
        i_peak = delta_v / z_mag

        self.lbl_throttle.setText(f"{int(throttle * 100)}%")
        self.lbl_status.setText(f"RPM: {rpm:.0f}\nFreq: {f_elec:.0f} Hz\nPeak I: {i_peak:.1f} A")

        # 2. Macro Sine Waves
        t_macro_ms = np.linspace(0, 2.0, 500)
        phase_u = omega_e * (t_macro_ms * 1e-3 + self.time_offset)
        phase_w = phase_u - (2 * np.pi / 3)

        i_u_sine = i_peak * np.sin(phase_u - z_angle)
        i_w_sine = i_peak * np.sin(phase_w - z_angle)

        self.line_real_u.set_data(t_macro_ms, i_u_sine)
        self.line_real_w.set_data(t_macro_ms, i_w_sine)

        # 3. Sampling
        t_sample_ms = np.arange(0, 2.0, 0.05)
        p_u_sample = omega_e * (t_sample_ms * 1e-3 + self.time_offset)
        p_w_sample = p_u_sample - (2 * np.pi / 3)

        i_u_real_sampled = i_peak * np.sin(p_u_sample - z_angle)
        i_w_real_sampled = i_peak * np.sin(p_w_sample - z_angle)

        # Quantize ADC1 (Clip to 0-3.3V)
        v_s_u = 1.65 + (i_u_real_sampled * R_SHUNT * GAIN)
        v_s_u = np.clip(v_s_u, 0, 3.3)
        v_q_u = np.round(v_s_u / ADC_LSB) * ADC_LSB
        i_meas_u = (v_q_u - 1.65) / (R_SHUNT * GAIN)

        # Quantize ADC2 (Clip to 0-3.3V)
        v_s_w = 1.65 + (i_w_real_sampled * R_SHUNT * GAIN)
        v_s_w = np.clip(v_s_w, 0, 3.3)
        v_q_w = np.round(v_s_w / ADC_LSB) * ADC_LSB
        i_meas_w = (v_q_w - 1.65) / (R_SHUNT * GAIN)

        offsets_u = np.column_stack((t_sample_ms, i_meas_u))
        offsets_w = np.column_stack((t_sample_ms, i_meas_w))
        self.scatter_u.set_offsets(offsets_u)
        self.scatter_w.set_offsets(offsets_w)

        max_val = max(i_peak * 1.5, 5)
        self.ax2.set_ylim(-max_val, max_val)

        # 4. Zoomed Plot (Instantaneous Voltage)
        t_zoom_us = np.linspace(0, 50, 1000)
        t_zoom_s = t_zoom_us * 1e-6

        i_u_zoom = i_peak * np.sin(omega_e * (self.time_offset + t_zoom_s) - z_angle)
        i_w_zoom = i_peak * np.sin(omega_e * (self.time_offset + t_zoom_s) - 2 * np.pi / 3 - z_angle)

        v_base_u = 1.65 + (i_u_zoom * R_SHUNT * GAIN)
        v_base_w = 1.65 + (i_w_zoom * R_SHUNT * GAIN)

        v_noisy_u = np.copy(v_base_u)
        v_noisy_w = np.copy(v_base_w)
        spike_mask = (t_zoom_us >= 0) & (t_zoom_us < 0.05)
        spike_val = 2.0 * np.sin(2 * np.pi * 10e6 * (t_zoom_us[spike_mask] * 1e-6))
        v_noisy_u[spike_mask] += spike_val
        v_noisy_w[spike_mask] += spike_val

        L1 = LEN_ADC1 * 1e-9
        C1 = LEN_ADC1 * 1e-12
        R1 = COPPER_RESISTIVITY * (LEN_ADC1 / 1000) / ((TRACE_WIDTH_MM / 1000) * COPPER_THICKNESS_M)
        L2 = LEN_ADC2 * 1e-9
        C2 = LEN_ADC2 * 1e-12
        R2 = COPPER_RESISTIVITY * (LEN_ADC2 / 1000) / ((TRACE_WIDTH_MM / 1000) * COPPER_THICKNESS_M)

        sys1 = self.build_state_space(L1, C1, R1)
        sys2 = self.build_state_space(L2, C2, R2)

        _, v_mcu1, _ = lsim(sys1, U=v_noisy_u, T=t_zoom_s, X0=[0, v_base_u[0], v_base_u[0]])
        _, v_mcu2, _ = lsim(sys2, U=v_noisy_w, T=t_zoom_s, X0=[0, v_base_w[0], v_base_w[0]])

        v_mcu1_clipped = np.clip(v_mcu1, 0, 3.3)
        v_mcu2_clipped = np.clip(v_mcu2, 0, 3.3)

        self.line_raw_u.set_data(t_zoom_us, np.clip(v_noisy_u, 0, 3.3))
        self.line_raw_w.set_data(t_zoom_us, np.clip(v_noisy_w, 0, 3.3))
        self.line_adc1.set_data(t_zoom_us, v_mcu1_clipped)
        self.line_adc2.set_data(t_zoom_us, v_mcu2_clipped)

        for coll in self.ax1.get_lines():
            if coll.get_linestyle() == ':':
                coll.set_xdata([sample_us, sample_us])

        # 5. Error
        err_u = i_meas_u - i_u_real_sampled
        err_w = i_meas_w - i_w_real_sampled

        self.line_err1.set_data(t_sample_ms, err_u)
        self.line_err2.set_data(t_sample_ms, err_w)

        max_err = max(np.max(np.abs(err_u)), np.max(np.abs(err_w)), 0.05)
        self.ax3.set_ylim(-max_err * 1.5, max_err * 1.5)

        self.canvas.draw_idle()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MotorADCApp()
    win.show()
    sys.exit(app.exec())