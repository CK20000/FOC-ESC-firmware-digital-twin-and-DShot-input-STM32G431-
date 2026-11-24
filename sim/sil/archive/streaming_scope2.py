"""
streaming_scope.py — live FOC scope with OpenGL, Autoscale, and Speed Control.
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

from sim.sil.archive.plant import PMSPlant
from sim.sil.archive.run_foc import load_firmware

COL_TIME = 0
COL_CCR1 = 1
COL_CCR2 = 2
COL_CCR3 = 3
COL_IA_TRUE = 4
COL_IB_TRUE = 5
COL_IU_FW = 6
COL_IV_FW = 7
COL_THETA_T = 8
COL_THETA_E = 9
COL_OMEGA_T = 10
COL_OMEGA_E = 11
COL_STATE = 12
COL_THROTTLE = 13
N_COLS = 14

RING_SIZE = 40000
STATE_NAMES = ["OFF", "ALIGN", "BLEND", "CLOSED", "KICK"]

class FOCScope(QMainWindow):
    def __init__(self, lib, plant):
        super().__init__()
        self.lib = lib
        self.plant = plant

        self.setWindowTitle("FOC SIL — live scope")
        self.resize(1400, 1200)

        pg.setConfigOptions(
            antialias=True,
            useOpenGL=True,
            background='#0a0a0a',
            foreground='#ddd'
        )

        self.ring = np.zeros((RING_SIZE, N_COLS), dtype=np.float32)
        self.total_cycles = 0

        self.playing = False
        self.cycles_per_frame = 500
        self.window_len = 800

        cw = QWidget()
        self.setCentralWidget(cw)
        layout = QVBoxLayout(cw)
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

        # --- RESTORED SPEED SLIDER ---
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

        self.lbl_state = QLabel("STATE: OFF  •  Ω_est:  0  •  Ω_true: 0  •  throttle: 0.00")
        self.lbl_state.setStyleSheet(
            "font-family: monospace; font-size: 13px; color: #00f0ff; "
            "background: #151515; padding: 6px; border: 1px solid #333;"
        )
        layout.addWidget(self.lbl_state)

        self.graphics = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphics)

        line_pen = lambda color, w=1.5: pg.mkPen(color, width=w, cosmetic=True)

        self.p_pwm = self.graphics.addPlot(row=0, col=0, title="PWM duty (CCR/4250)")
        self.p_pwm.setYRange(-0.05, 1.05)
        self.c_ccr1 = self.p_pwm.plot(pen=line_pen('#00f0ff'))
        self.c_ccr2 = self.p_pwm.plot(pen=line_pen('#ff0055'))
        self.c_ccr3 = self.p_pwm.plot(pen=line_pen('#ffcc00'))

        self.p_i = self.graphics.addPlot(row=1, col=0, title="Phase Currents (A)")
        self.c_ia = self.p_i.plot(pen=line_pen('#00f0ff'))
        self.c_ib = self.p_i.plot(pen=line_pen('#ff0055'))
        self.c_iu = self.p_i.plot(pen=line_pen('#888', 1.0))

        self.p_theta = self.graphics.addPlot(row=2, col=0, title="Rotor angle θ (rad)")
        self.p_theta.setYRange(-0.2, 6.5)
        self.c_th_t = self.p_theta.plot(pen=line_pen('#888'))
        self.c_th_e = self.p_theta.plot(pen=line_pen('#ffb733'))

        self.p_om = self.graphics.addPlot(row=3, col=0, title="Rotor speed ω (rad/s)")
        self.p_om.setYRange(-15000, 15000)
        self.c_om_t = self.p_om.plot(pen=line_pen('#888'))
        self.c_om_e = self.p_om.plot(pen=line_pen('#00ff88'))

        self.p_idq = self.graphics.addPlot(row=4, col=0, title="D/Q Currents (A)")
        self.p_idq.setYRange(-50, 50)
        self.c_id = self.p_idq.plot(pen=line_pen('#ff00ff', 2.0))
        self.c_iq = self.p_idq.plot(pen=line_pen('#00ff00', 2.0))

        self.p_i.setXLink(self.p_pwm)
        self.p_theta.setXLink(self.p_pwm)
        self.p_om.setXLink(self.p_pwm)
        self.p_idq.setXLink(self.p_pwm)

        self.timer = QTimer()
        self.timer.setInterval(30)
        self.timer.timeout.connect(self.on_tick)

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
            ccr1 = lib.sil_tim1_get_ccr1()
            ccr2 = lib.sil_tim1_get_ccr2()
            ccr3 = lib.sil_tim1_get_ccr3()

            plant.step(ccr1, ccr2, ccr3)

            cnt_u, cnt_v = plant.adc_counts()

            lib.sil_adc1_set_jdr1(cnt_u)
            lib.sil_adc2_set_jdr1(cnt_v)
            lib.sil_tim2_advance(3541)
            lib.foc_loop()

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

        if self.total_cycles < RING_SIZE:
            data = self.ring[self.total_cycles - N:self.total_cycles]
        else:
            start = (self.total_cycles - N) % RING_SIZE
            end = self.total_cycles % RING_SIZE
            if start < end:
                data = self.ring[start:end]
            else:
                data = np.vstack([self.ring[start:], self.ring[:end]])

        data = np.nan_to_num(data, nan=0.0, posinf=0.0, neginf=0.0)
        x = data[:, COL_TIME] - data[0, COL_TIME]
        ccr_scale = 1.0 / 1770.0

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

        i_u = data[:, COL_IU_FW]
        i_v = data[:, COL_IV_FW]
        i_alpha = i_u
        i_beta = (i_u + 2.0 * i_v) * 0.577350269

        omega_e = data[:, COL_OMEGA_E]
        advanced_angle = data[:, COL_THETA_E] + omega_e * 40e-6

        cos_a = np.cos(advanced_angle)
        sin_a = np.sin(advanced_angle)
        id_calc = i_alpha * cos_a + i_beta * sin_a
        iq_calc = -i_alpha * sin_a + i_beta * cos_a

        self.c_id.setData(x, id_calc)
        self.c_iq.setData(x, iq_calc)

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

            idq_max = max(5.0, 1.2 * max(
                np.abs(id_calc).max(),
                np.abs(iq_calc).max()
            ))
            self.p_idq.setYRange(-idq_max, idq_max, padding=0)

            pwm_min = min(data[:, COL_CCR1].min(), data[:, COL_CCR2].min(), data[:, COL_CCR3].min()) * ccr_scale
            pwm_max = max(data[:, COL_CCR1].max(), data[:, COL_CCR2].max(), data[:, COL_CCR3].max()) * ccr_scale
            self.p_pwm.setYRange(max(-0.05, pwm_min - 0.05), min(1.05, pwm_max + 0.05), padding=0)

        last = data[-1]
        st = int(last[COL_STATE])
        st_name = STATE_NAMES[st] if 0 <= st < len(STATE_NAMES) else "?"
        self.lbl_state.setText(
            f"STATE: {st_name:6s}   "
            f"Ω_est: {last[COL_OMEGA_E]:+7.0f} rad/s   "
            f"Ω_true: {last[COL_OMEGA_T]:+7.0f} rad/s   "
            f"θ_t: {last[COL_THETA_T]:4.2f}   θ_e: {last[COL_THETA_E]:4.2f}   "
            f"throttle: {last[COL_THROTTLE]:.2f}   "
            f"Iq: {iq_calc[-1]:.1f}A   "
            f"cycle: {int(last[COL_TIME])}"
        )

def main():
    so_path = Path(__file__).parent / "foc_sim.so"
    if not so_path.exists():
        print(f"[!] {so_path} not found — run `make` first")
        sys.exit(1)

    lib = load_firmware(so_path)
    plant = PMSPlant(
        RS=0.110, LS=30e-6, LAMBDA=0.00041,
        POLE_PAIRS=7, INERTIA=5e-6, V_BUS=25.2,
        BETA_SIGN=+1, DAMPING=2e-5,
    )

    app = QApplication(sys.argv)
    win = FOCScope(lib, plant)
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()