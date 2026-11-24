"""
streaming_scope.py — live FOC scope.

Runs firmware .so in a loop, plots rolling window of last N cycles.
Throttle slider, play/pause, speed control, reset.
"""

import sys
import numpy as np
from pathlib import Path

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QSlider, QLabel, QPushButton, QCheckBox
)
from PyQt6.QtCore import Qt, QTimer
import pyqtgraph as pg

from plant import PMSPlant
from run_foc import load_firmware


COL_TIME   = 0
COL_CCR1   = 1
COL_CCR2   = 2
COL_CCR3   = 3
COL_IA_TRUE = 4
COL_IB_TRUE = 5
COL_IU_FW   = 6
COL_IV_FW   = 7
COL_THETA_T = 8
COL_THETA_E = 9
COL_OMEGA_T = 10
COL_OMEGA_E = 11
COL_STATE   = 12
COL_THROTTLE = 13
N_COLS = 14

RING_SIZE = 40000   # 2 sec of history
STATE_NAMES = ["OFF", "ALIGN", "RAMP", "BLEND", "CLOSED"]


class FOCScope(QMainWindow):
    def __init__(self, lib, plant):
        super().__init__()
        self.lib = lib
        self.plant = plant

        self.setWindowTitle("FOC SIL — live scope")
        self.resize(1400, 900)
        pg.setConfigOptions(antialias=False, background='#0a0a0a', foreground='#ddd')

        # Ring buffer
        self.ring = np.zeros((RING_SIZE, N_COLS), dtype=np.float32)
        self.total_cycles = 0

        # State
        self.playing = False
        self.cycles_per_frame = 500     # speed
        self.window_len = 800           # visible cycles

        # --- Central widget ---
        cw = QWidget()
        self.setCentralWidget(cw)
        layout = QVBoxLayout(cw)

        # --- Control bar ---
        ctrl = QHBoxLayout()

        self.btn_play = QPushButton("▶ Play")
        self.btn_play.setCheckable(True)
        self.btn_play.setStyleSheet("font-size:16px; padding:4px 16px;")
        self.btn_play.clicked.connect(self.toggle_play)
        ctrl.addWidget(self.btn_play)

        self.btn_reset = QPushButton("⟲ Reset")
        self.btn_reset.setStyleSheet("font-size:16px; padding:4px 16px;")
        self.btn_reset.clicked.connect(self.reset_sim)
        ctrl.addWidget(self.btn_reset)

        ctrl.addWidget(QLabel("Throttle:"))
        self.sl_throttle = QSlider(Qt.Orientation.Horizontal)
        self.sl_throttle.setRange(0, 1000)
        self.sl_throttle.setValue(0)
        self.sl_throttle.setMinimumWidth(180)
        self.sl_throttle.valueChanged.connect(self.on_throttle)
        ctrl.addWidget(self.sl_throttle)
        self.lbl_throttle = QLabel("0.00")
        self.lbl_throttle.setStyleSheet("font-weight:bold; color:#ffb733; font-size:14px; min-width:50px;")
        ctrl.addWidget(self.lbl_throttle)

        ctrl.addStretch()

        ctrl.addWidget(QLabel("Speed:"))
        self.sl_speed = QSlider(Qt.Orientation.Horizontal)
        self.sl_speed.setRange(20, 3000)
        self.sl_speed.setValue(500)
        self.sl_speed.setMinimumWidth(140)
        self.sl_speed.valueChanged.connect(self.on_speed)
        ctrl.addWidget(self.sl_speed)
        self.lbl_speed = QLabel("500/f")
        self.lbl_speed.setStyleSheet("color:#888; min-width:60px;")
        ctrl.addWidget(self.lbl_speed)

        ctrl.addWidget(QLabel("Window:"))
        self.sl_window = QSlider(Qt.Orientation.Horizontal)
        self.sl_window.setRange(100, 5000)
        self.sl_window.setValue(800)
        self.sl_window.setMinimumWidth(140)
        self.sl_window.valueChanged.connect(self.on_window)
        ctrl.addWidget(self.sl_window)
        self.lbl_window = QLabel("800c")
        self.lbl_window.setStyleSheet("color:#888; min-width:60px;")
        ctrl.addWidget(self.lbl_window)

        self.cb_follow = QCheckBox("Autoscale")
        self.cb_follow.setChecked(True)
        ctrl.addWidget(self.cb_follow)

        layout.addLayout(ctrl)

        # --- Status label ---
        self.lbl_state = QLabel("STATE: OFF  •  Ω_est:  0  •  Ω_true: 0  •  throttle: 0.00")
        self.lbl_state.setStyleSheet(
            "font-family: monospace; font-size: 13px; color: #00f0ff; "
            "background: #151515; padding: 6px; border: 1px solid #333;"
        )
        layout.addWidget(self.lbl_state)

        # --- Plots ---
        self.graphics = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphics)

        # Row 1: PWM duty
        self.p_pwm = self.graphics.addPlot(row=0, col=0, title="PWM duty (CCR/4250)")
        self.p_pwm.setLabel('left', 'Duty')
        self.p_pwm.showGrid(x=True, y=True, alpha=0.15)
        self.p_pwm.setYRange(-0.05, 1.05)
        self.c_ccr1 = self.p_pwm.plot(pen=pg.mkPen('#00f0ff', width=1))
        self.c_ccr2 = self.p_pwm.plot(pen=pg.mkPen('#ff0055', width=1))
        self.c_ccr3 = self.p_pwm.plot(pen=pg.mkPen('#ffcc00', width=1))

        # Row 2: Currents
        self.p_i = self.graphics.addPlot(row=1, col=0, title="Currents (A)")
        self.p_i.setLabel('left', 'Amps')
        self.p_i.showGrid(x=True, y=True, alpha=0.15)
        self.c_ia  = self.p_i.plot(pen=pg.mkPen('#00f0ff', width=1))
        self.c_ib  = self.p_i.plot(pen=pg.mkPen('#ff0055', width=1))
        self.c_iu  = self.p_i.plot(pen=pg.mkPen('#888', width=1, style=Qt.PenStyle.DotLine))
        self.c_iu.setData([], [])  # firmware view

        # Row 3: Theta
        self.p_theta = self.graphics.addPlot(row=2, col=0, title="Rotor angle θ (rad)")
        self.p_theta.setLabel('left', 'rad')
        self.p_theta.showGrid(x=True, y=True, alpha=0.15)
        self.p_theta.setYRange(-0.2, 6.5)
        self.c_th_t = self.p_theta.plot(pen=pg.mkPen('#888', width=1))
        self.c_th_e = self.p_theta.plot(pen=pg.mkPen('#ffb733', width=1))

        # Row 4: Omega
        self.p_om = self.graphics.addPlot(row=3, col=0, title="Rotor speed ω (rad/s)")
        self.p_om.setLabel('left', 'rad/s')
        self.p_om.setLabel('bottom', 'cycle')
        self.p_om.showGrid(x=True, y=True, alpha=0.15)
        self.p_om.setYRange(-15000, 15000)
        self.c_om_t = self.p_om.plot(pen=pg.mkPen('#888', width=1))
        self.c_om_e = self.p_om.plot(pen=pg.mkPen('#00ff88', width=1))

        # Link X axes (all plots scroll together)
        self.p_i.setXLink(self.p_pwm)
        self.p_theta.setXLink(self.p_pwm)
        self.p_om.setXLink(self.p_pwm)

        # Timer
        self.timer = QTimer()
        self.timer.setInterval(30)   # 33 FPS
        self.timer.timeout.connect(self.on_tick)

        # Init firmware
        self._firmware_init()

    def _firmware_init(self):
        self.lib.foc_set_adc_offsets(2048.0, 2048.0)
        self.lib.foc_setup()
        self.lib.sil_dshot_set_armed(1)

    def toggle_play(self):
        self.playing = self.btn_play.isChecked()
        if self.playing:
            self.btn_play.setText("⏸ Pause")
            self.timer.start()
        else:
            self.btn_play.setText("▶ Play")
            self.timer.stop()

    def on_throttle(self):
        val = self.sl_throttle.value() / 1000.0
        self.lbl_throttle.setText(f"{val:.2f}")
        self.lib.sil_dshot_set_throttle(float(val))

    def on_speed(self):
        self.cycles_per_frame = self.sl_speed.value()
        self.lbl_speed.setText(f"{self.cycles_per_frame}/f")

    def on_window(self):
        self.window_len = self.sl_window.value()
        self.lbl_window.setText(f"{self.window_len}c")

    def reset_sim(self):
        self.timer.stop()
        self.btn_play.setChecked(False)
        self.btn_play.setText("▶ Play")
        self.playing = False

        self.ring.fill(0)
        self.total_cycles = 0

        # Reset plant
        self.plant.__init__(
            RS=0.090, LS=30e-6, LAMBDA=0.00041,
            POLE_PAIRS=7, INERTIA=5e-6, V_BUS=25.2,
            BETA_SIGN=+1, DAMPING=2e-5,
        )

        self._firmware_init()
        self._redraw()

    def on_tick(self):
        lib = self.lib
        plant = self.plant
        N = self.cycles_per_frame

        for _ in range(N):
            # --- Read what firmware wrote ---
            ccr1 = lib.sil_tim1_get_ccr1()
            ccr2 = lib.sil_tim1_get_ccr2()
            ccr3 = lib.sil_tim1_get_ccr3()

            # --- Plant step ---
            plant.step(ccr1, ccr2, ccr3)

            # --- Feed currents to firmware ---
            cnt_u, cnt_v = plant.adc_counts()
            lib.sil_adc1_set_jdr1(cnt_u)
            lib.sil_adc2_set_jdr1(cnt_v)

            # --- Advance time + run firmware ---
            lib.sil_tim2_advance(567)
            lib.foc_loop()

            # --- Read back state ---
            row = np.array([
                self.total_cycles,
                ccr1, ccr2, ccr3,
                plant.i_alpha, plant.i_beta,
                lib.foc_get_phase_u_current(),
                lib.foc_get_phase_v_current(),
                plant.theta,
                lib.foc_get_estimated_angle(),
                plant.omega,
                lib.foc_get_estimated_speed(),
                float(lib.foc_get_state()),
                float(lib.sil_dshot_get_throttle()),
            ], dtype=np.float32)

            self.ring[self.total_cycles % RING_SIZE] = row
            self.total_cycles += 1

        self._redraw()

    def _redraw(self):
        if self.total_cycles == 0:
            return

        N = min(self.window_len, self.total_cycles)
        if N < 2:
            return

        # Extract last N cycles from ring buffer
        if self.total_cycles < RING_SIZE:
            data = self.ring[self.total_cycles - N:self.total_cycles]
        else:
            start = (self.total_cycles - N) % RING_SIZE
            end = self.total_cycles % RING_SIZE
            if start < end:
                data = self.ring[start:end]
            else:
                data = np.vstack([self.ring[start:], self.ring[:end]])

        # --- FIX PYQT WARNINGS: Scrub NaN and Infinity ---
        data = np.nan_to_num(data, nan=0.0, posinf=0.0, neginf=0.0)

        x = data[:, COL_TIME] - data[0, COL_TIME]
        ccr_scale = 1.0 / 4250.0

        self.c_ccr1.setData(x, data[:, COL_CCR1] * ccr_scale)
        self.c_ccr2.setData(x, data[:, COL_CCR2] * ccr_scale)
        self.c_ccr3.setData(x, data[:, COL_CCR3] * ccr_scale)

        self.c_ia.setData(x, data[:, COL_IA_TRUE])
        self.c_ib.setData(x, data[:, COL_IB_TRUE])
        self.c_iu.setData(x, data[:, COL_IU_FW])

        self.c_th_t.setData(x, data[:, COL_THETA_T])
        self.c_th_e.setData(x, data[:, COL_THETA_E])

        self.c_om_t.setData(x, data[:, COL_OMEGA_T])
        self.c_om_e.setData(x, data[:, COL_OMEGA_E])

        # Autoscale Y for current + omega plots
        if self.cb_follow.isChecked():
            i_max = max(2.0, 1.2 * max(
                np.abs(data[:, COL_IA_TRUE]).max(),
                np.abs(data[:, COL_IB_TRUE]).max(),
                np.abs(data[:, COL_IU_FW]).max(),
            ))
            self.p_i.setYRange(-i_max, i_max, padding=0)

            om_max = max(50.0, 1.2 * max(
                np.abs(data[:, COL_OMEGA_T]).max(),
                np.abs(data[:, COL_OMEGA_E]).max(),
            ))
            self.p_om.setYRange(-om_max, om_max, padding=0)

        # Status label
        last = data[-1]
        st = int(last[COL_STATE])
        st_name = STATE_NAMES[st] if 0 <= st < len(STATE_NAMES) else "?"
        self.lbl_state.setText(
            f"STATE: {st_name:6s}   "
            f"Ω_est: {last[COL_OMEGA_E]:+7.0f} rad/s   "
            f"Ω_true: {last[COL_OMEGA_T]:+7.0f} rad/s   "
            f"θ_t: {last[COL_THETA_T]:4.2f}   θ_e: {last[COL_THETA_E]:4.2f}   "
            f"throttle: {last[COL_THROTTLE]:.2f}   "
            f"cycle: {int(last[COL_TIME])}"
        )


def main():
    so_path = Path(__file__).parent / "foc_sim.so"
    if not so_path.exists():
        print(f"[!] {so_path} not found — run `make` first")
        sys.exit(1)

    lib = load_firmware(so_path)
    plant = PMSPlant(
        RS=0.090, LS=30e-6, LAMBDA=0.00041,
        POLE_PAIRS=7, INERTIA=5e-6, V_BUS=25.2,
        BETA_SIGN=+1, DAMPING=2e-5,
    )

    app = QApplication(sys.argv)
    win = FOCScope(lib, plant)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()