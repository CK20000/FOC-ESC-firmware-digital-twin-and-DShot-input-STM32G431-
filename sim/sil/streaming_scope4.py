"""
streamingscope4.py — C-Plant SIL Hybrid
Python is used for UI, Thermal (Conduction + Switching), and Ground Bounce Extraction.
The 48kHz electromagnetic loop runs purely in C.
"""
import sys
import os
import json
import ctypes
import numpy as np
from pathlib import Path

# ── Path Setup: Bridge SIL to HIL ───────────────────────────────────────
base = Path(__file__).parent.resolve()
hil_dir = base.parent / "hil"

if hil_dir.exists():
    sys.path.insert(0, str(hil_dir))

try:
    from unified_esc_dashboard import TransientSolver, LAYERS
    from ground_bounce_analyzer import GroundBounceTab
except ImportError as e:
    print(f"[!] Failed to import from {hil_dir}: {e}")
    sys.exit(1)

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QSlider, QLabel, QPushButton, QCheckBox, QTabWidget, QSizeGrip
)
from PyQt6.QtCore import Qt, QRectF, QTimer
import pyqtgraph as pg
from pyqtgraph import ColorBarItem

pg.setConfigOptions(antialias=True, useOpenGL=True, background='#0a0a0a', foreground='#ddd')

COL_TIME, COL_CCR1, COL_CCR2, COL_CCR3 = 0, 1, 2, 3
COL_IA_TRUE, COL_IB_TRUE = 4, 5
COL_IU_FW, COL_IV_FW = 6, 7
COL_THETA_T, COL_THETA_E = 8, 9
COL_OMEGA_T, COL_OMEGA_E = 10, 11
COL_STATE, COL_THROTTLE = 12, 13
COL_VBUS = 14
N_COLS = 15
RING_SIZE = 40000
STATE_NAMES = ["OFF", "ALIGN", "BLEND", "CLOSED", "KICK", "FAULT"]

STYLESHEET = """
QMainWindow { background-color: #222222; }
QWidget { background-color: #050505; color: #e8e8e8; }
QTabWidget::pane { border: 1px solid #333333; background-color: #050505; }
QTabBar::tab { background-color: #111111; color: #888888; padding: 8px 20px; border: 1px solid #333333; border-bottom: none; font-size: 10pt; }
QTabBar::tab:selected { background-color: #050505; color: #ffffff; border-top: 2px solid #ffb733; }
QTabBar::tab:hover { background-color: #222222; }
QLabel { color: #e8e8e8; background: transparent; }
QCheckBox { color: #e8e8e8; spacing: 8px; background: transparent; }
QSlider::groove:horizontal { border: 1px solid #333; height: 6px; background: #111; margin: 2px 0; border-radius: 3px; }
QSlider::handle:horizontal { background: #ffb733; border: 1px solid #ffb733; width: 14px; margin: -5px 0; border-radius: 7px; }
QSlider::handle:horizontal:hover { background: #ffcc66; }
QPushButton { background-color: #222222; color: #ffffff; border: 1px solid #555555; border-radius: 4px; }
QPushButton:hover { background-color: #333333; }
QPushButton:checked { background-color: #ffb733; color: #000000; border: 1px solid #ffb733; }
QSizeGrip { background-color: transparent; width: 15px; height: 15px; }
"""

class TitleBar(QWidget):
    def __init__(self, parent=None, title="FOC SIL — C-Plant Hybrid"):
        super().__init__(parent)
        self.parent = parent
        self.setFixedHeight(35)
        self.setStyleSheet("background-color: #222222;")

        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 0, 10, 0)

        self.title_label = QLabel(title)
        self.title_label.setStyleSheet("color: #ffb733; font-weight: bold; font-size: 11pt; background: transparent;")
        layout.addWidget(self.title_label)

        layout.addStretch()

        self.btn_min = QPushButton("—")
        self.btn_min.setFixedSize(30, 30)
        self.btn_min.setStyleSheet("""
            QPushButton { background-color: transparent; color: #888; border: none; font-size: 18px; font-weight: bold; }
            QPushButton:hover { background-color: #444; color: white; border-radius: 4px; }
        """)
        self.btn_min.clicked.connect(self.parent.showMinimized)
        layout.addWidget(self.btn_min)

        self.btn_max = QPushButton("☐")
        self.btn_max.setFixedSize(30, 30)
        self.btn_max.setStyleSheet("""
            QPushButton { background-color: transparent; color: #888; border: none; font-size: 18px; font-weight: bold; }
            QPushButton:hover { background-color: #444; color: white; border-radius: 4px; }
        """)
        self.btn_max.clicked.connect(self._toggle_max)
        layout.addWidget(self.btn_max)

        self.btn_close = QPushButton("✕")
        self.btn_close.setFixedSize(30, 30)
        self.btn_close.setStyleSheet("""
            QPushButton { background-color: transparent; color: #888; border: none; font-size: 14px; font-weight: bold; }
            QPushButton:hover { background-color: #ff0055; color: white; border-radius: 4px; }
        """)
        self.btn_close.clicked.connect(self.parent.close)
        layout.addWidget(self.btn_close)

    def _toggle_max(self):
        if self.parent.isMaximized():
            self.parent.showNormal()
        else:
            self.parent.showMaximized()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.start_pos = event.globalPosition().toPoint() - self.parent.frameGeometry().topLeft()
            event.accept()

    def mouseMoveEvent(self, event):
        if self.start_pos is not None and event.buttons() == Qt.MouseButton.LeftButton:
            self.parent.move(event.globalPosition().toPoint() - self.start_pos)
            event.accept()

    def mouseReleaseEvent(self, event):
        self.start_pos = None
        event.accept()

def load_c_sim(so_path):
    lib = ctypes.CDLL(str(so_path))
    lib.sim_create.restype = ctypes.c_void_p
    lib.sim_create.argtypes = [ctypes.c_uint32, ctypes.POINTER(ctypes.c_float)]
    lib.sim_run.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.sim_set_throttle.argtypes = [ctypes.c_void_p, ctypes.c_float]
    lib.sim_get_thermal_tick.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float)]
    lib.sim_get_thermal_tick.restype = ctypes.c_int
    lib.sim_reset.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    lib.sim_destroy.argtypes = [ctypes.c_void_p]

    lib.sim_set_gnd_bounce.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]
    lib.sim_set_stall.argtypes = [ctypes.c_void_p, ctypes.c_int]
    return lib

class FOCScope(QMainWindow):
    def __init__(self, geom, so_path):
        super().__init__()
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint)
        self.resize(1400, 1200)

        self.cycles_per_frame = 1440
        self.buffer = np.zeros((self.cycles_per_frame, N_COLS), dtype=np.float32)
        self.lib = load_c_sim(so_path)
        self.session = self.lib.sim_create(42, self.buffer.ctypes.data_as(ctypes.POINTER(ctypes.c_float)))

        # ── DYNAMIC GROUND BOUNCE EXTRACTION ──
        print("[*] Initializing 3D Ground Bounce Matrix (2-5s)...")
        self.gnd_solver_app = GroundBounceTab(geom)

        # Remove Solder Highways tab
        if self.gnd_solver_app.tabs.count() > 1:
            self.gnd_solver_app.tabs.removeTab(1)

        b = np.zeros(self.gnd_solver_app.N)
        for node in self.gnd_solver_app.fet_nodes: b[node] += 1.0 / len(self.gnd_solver_app.fet_nodes)
        for node in self.gnd_solver_app.tp5_nodes: b[node] -= 1.0 / len(self.gnd_solver_app.tp5_nodes)

        V = self.gnd_solver_app.lu.solve(b)
        v_ref = np.mean([V[n] for n in self.gnd_solver_app.tp5_nodes])
        V = V - v_ref

        nz, ny, nx = self.gnd_solver_app.nz, self.gnd_solver_app.ny, self.gnd_solver_app.nx

        # --- BOUNDING BOX EXTRACTION (10x10mm) ---
        mcu_nodes = []
        adc_nodes = []

        for fp in self.gnd_solver_app.geom.get("footprints", []):
            val = (fp.get("value") or "").upper()
            ref = fp["ref"].upper()

            # Identify MCU and AD8418 by value or reference
            is_mcu = "STM32" in val or "G431" in val or ref in ("U1", "U2")
            is_adc = "AD8418" in val or "8418" in val or ref in ("U3", "U4", "U5")

            if is_mcu or is_adc:
                cx, cy = fp["x_mm"], fp["y_mm"]
                z = 0 if fp.get("layer") == "F_Cu" else 5

                # 10x10mm box -> +/- 5mm -> +/- 10 grid cells (0.5mm each)
                j_center = int((cx - self.gnd_solver_app.bx) / 0.5)
                i_center = int((cy - self.gnd_solver_app.by) / 0.5)

                for di in range(-10, 11):
                    for dj in range(-10, 11):
                        i = i_center + di
                        j = j_center + dj
                        if 0 <= i < self.gnd_solver_app.ny and 0 <= j < self.gnd_solver_app.nx:
                            if self.gnd_solver_app.tensor[z][i][j]["copper"] > 0.1:
                                node = z * (
                                            self.gnd_solver_app.ny * self.gnd_solver_app.nx) + i * self.gnd_solver_app.nx + j
                                if is_mcu: mcu_nodes.append(node)
                                if is_adc: adc_nodes.append(node)

        mcu_nodes = list(set(mcu_nodes))
        adc_nodes = list(set(adc_nodes))

        # Extract the average voltage inside the boxes
        if mcu_nodes:
            self.r_mcu = abs(float(np.mean([V[n] for n in mcu_nodes])))
        else:
            self.r_mcu = 0.0005  # Fallback

        if adc_nodes:
            self.r_adc = abs(float(np.mean([V[n] for n in adc_nodes])))
        else:
            self.r_adc = 0.0005  # Fallback

        # Safety cap to prevent crazy values if the bounding box hit empty copper
        if self.r_mcu > 0.002: self.r_mcu = 0.0005
        if self.r_adc > 0.002: self.r_adc = 0.0005

        print(f"[+] Dynamic Extraction: MCU Side {self.r_mcu*1000:.2f} mOhm, ADC Side {self.r_adc*1000:.2f} mOhm")
        self.lib.sim_set_gnd_bounce(self.session, ctypes.c_float(self.r_adc), ctypes.c_float(self.r_mcu))

        # ── PYTHON THERMAL SOLVER ──
        initial_powers = {"U1": 0.02, "U2": 0.165, "U3": 0.183, "U4": 0.02, "U5": 0.158, "U6": 0.15, "U9": 0.107, "R22": 0.0, "R23": 0.0}
        for i in range(1, 13): initial_powers[f"Q{i}"] = 0.0
        print("[*] Factorizing thermal matrix… (2–5 s)")
        self.thermal = TransientSolver(geom, initial_powers, h_conv=15.0, h_sink=150.0)
        self.last_T_fields = None
        self.max_T_fet = 25.0
        self.thermal_version = 0
        self.i2_avg = (ctypes.c_float * 3)()

        # ── UI Setup ──
        self.ring = np.zeros((RING_SIZE, N_COLS), dtype=np.float32)
        self.total_cycles = 0
        self.playing = False
        self.window_len = 800
        self._last_thermal_version = -1

        self._build_ui()
        self.timer = QTimer()
        self.timer.setInterval(30)
        self.timer.timeout.connect(self.on_tick)

    def _build_ui(self):
        cw = QWidget()
        self.setCentralWidget(cw)

        outer_layout = QVBoxLayout(cw)
        outer_layout.setContentsMargins(2, 2, 2, 2)
        outer_layout.setSpacing(0)

        self.title_bar = TitleBar(self)
        outer_layout.addWidget(self.title_bar)

        content_widget = QWidget()
        layout = QVBoxLayout(content_widget)
        layout.setContentsMargins(10, 10, 10, 10)

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
        self.sl_speed.setRange(20, 5000)
        self.sl_speed.setValue(1440)
        self.sl_speed.setMinimumWidth(140)
        self.sl_speed.valueChanged.connect(self.on_speed)
        ctrl.addWidget(self.sl_speed)
        self.lbl_speed = QLabel("1440/f")
        self.lbl_speed.setStyleSheet("color:#888; min-width:70px;")
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

        self.cb_stall = QCheckBox("Stall Motor")
        self.cb_stall.setStyleSheet("color: #ff0055; font-weight: bold;")
        self.cb_stall.stateChanged.connect(self.on_stall_toggle)
        ctrl.addWidget(self.cb_stall)

        self.cb_follow = QCheckBox("Autoscale")
        self.cb_follow.setChecked(True)
        ctrl.addWidget(self.cb_follow)

        layout.addLayout(ctrl)

        self.lbl_state = QLabel("STATE: OFF  •  Ω_est:  0  •  Ω_true: 0  •  throttle: 0.00")
        self.lbl_state.setStyleSheet("font-family: monospace; font-size: 13px; color: #00f0ff; background: #151515; padding: 6px; border: 1px solid #333;")
        layout.addWidget(self.lbl_state)

        self.graphics = pg.GraphicsLayoutWidget()
        self._build_scope_plots()

        scope_widget = QWidget()
        scope_layout = QVBoxLayout(scope_widget)
        scope_layout.setContentsMargins(0, 0, 0, 0)
        scope_layout.addWidget(self.lbl_state)
        scope_layout.addWidget(self.graphics)

        self.tabs = QTabWidget()
        self.tabs.addTab(scope_widget, "Scope")
        self.tabs.addTab(self._build_thermal_tab(), "PCB Thermal")

        # ── Extract the heatmap directly to avoid nested tabs ──
        heatmap_widget = self.gnd_solver_app.bounce_tab
        self.tabs.addTab(heatmap_widget, "GND Bounce Map")

        layout.addWidget(self.tabs)

        bottom_bar = QHBoxLayout()
        bottom_bar.setContentsMargins(0, 0, 0, 0)
        bottom_bar.addStretch()
        bottom_bar.addWidget(QSizeGrip(cw))
        layout.addLayout(bottom_bar)

        # Hide the manual slider inside the heatmap tab since it's auto-driven
        self.gnd_solver_app.slider.hide()
        self.gnd_solver_app.lbl_current.hide()

        outer_layout.addWidget(content_widget)

    def _build_scope_plots(self):
        pen = lambda c, w=1.5: pg.mkPen(c, width=w, cosmetic=True)
        self.p_pwm = self.graphics.addPlot(row=0, col=0, title="PWM duty (CCR/1770)")
        self.p_pwm.setYRange(-0.05, 1.05)
        self.c_ccr1 = self.p_pwm.plot(pen=pen('#00f0ff'))
        self.c_ccr2 = self.p_pwm.plot(pen=pen('#ff0055'))
        self.c_ccr3 = self.p_pwm.plot(pen=pen('#ffcc00'))

        self.p_i = self.graphics.addPlot(row=1, col=0, title="Phase Currents (A)")
        self.c_ia = self.p_i.plot(pen=pen('#00f0ff'))
        self.c_ib = self.p_i.plot(pen=pen('#ff0055'))
        self.c_iu = self.p_i.plot(pen=pen('#888', 1.0))

        self.p_theta = self.graphics.addPlot(row=2, col=0, title="Rotor angle θ (rad)")
        self.p_theta.setYRange(-0.2, 6.5)
        self.c_th_t = self.p_theta.plot(pen=pen('#888'))
        self.c_th_e = self.p_theta.plot(pen=pen('#ffb733'))

        self.p_om = self.graphics.addPlot(row=3, col=0, title="Rotor speed ω (rad/s)")
        self.p_om.setYRange(-15000, 15000)
        self.c_om_t = self.p_om.plot(pen=pen('#888'))
        self.c_om_e = self.p_om.plot(pen=pen('#00ff88'))

        self.p_idq = self.graphics.addPlot(row=4, col=0, title="D/Q Currents (A)")
        self.p_idq.setYRange(-50, 50)
        self.c_id = self.p_idq.plot(pen=pen('#ff00ff', 2.0))
        self.c_iq = self.p_idq.plot(pen=pen('#00ff00', 2.0))

        self.p_gnd = self.graphics.addPlot(row=5, col=0, title="ADC Ground Bounce (mV)")
        self.p_gnd.setYRange(0, 50)
        self.c_gnd = self.p_gnd.plot(pen=pen('#ffaa00', 2.0))

        self.p_vbus = self.graphics.addPlot(row=6, col=0, title="Bus Voltage (V)")
        self.p_vbus.setYRange(15, 27)
        self.c_vbus = self.p_vbus.plot(pen=pen('#ff00aa', 2.0))

        for p in (self.p_i, self.p_theta, self.p_om, self.p_idq, self.p_gnd, self.p_vbus):
            p.setXLink(self.p_pwm)

    def _build_thermal_tab(self):
        o = self.thermal.geom["outline"]
        self._thermal_rect = QRectF(o["x_mm"], o["y_mm"], o["w_mm"], o["h_mm"])
        w = QWidget()
        v = QVBoxLayout(w)
        self.lbl_thermal = QLabel("T_FET: 25.0 °C | waiting for tick…")
        self.lbl_thermal.setStyleSheet("font-family: monospace; font-size: 13px; color: #ffb733; background: #151515; padding: 6px; border: 1px solid #333;")
        v.addWidget(self.lbl_thermal)

        g = pg.GraphicsLayoutWidget()
        v.addWidget(g)
        self._thermal_cmap = pg.colormap.get('turbo')
        self._thermal_lut = self._thermal_cmap.getLookupTable(nPts=256)
        self._thermal_imgs = {}
        for row in range(2):
            for col in range(3):
                p = g.addPlot(row=row, col=col)
                p.setAspectLocked(True); p.invertY(True); p.hideAxis('left'); p.hideAxis('bottom')
                p.setRange(self._thermal_rect, padding=0.05)
                p.setTitle(LAYERS[row*3 + col])
                img = pg.ImageItem()
                p.addItem(img)
                self._thermal_imgs[row*3 + col] = img

        self._thermal_cbar = ColorBarItem(values=(25.0, 95.0), colorMap=self._thermal_cmap)
        self._thermal_cbar.setImageItem([self._thermal_imgs[i] for i in range(6)])
        g.addItem(self._thermal_cbar, row=0, col=3, rowspan=2)
        return w

    def on_tick(self):
        N = self.cycles_per_frame
        self.lib.sim_run(self.session, N)

        start_idx = self.total_cycles % RING_SIZE
        end_idx = start_idx + N
        if end_idx <= RING_SIZE:
            self.ring[start_idx:end_idx] = self.buffer[:N]
        else:
            wrap = RING_SIZE - start_idx
            self.ring[start_idx:] = self.buffer[:wrap]
            self.ring[:end_idx - RING_SIZE] = self.buffer[wrap:N]
        self.total_cycles += N

        if self.lib.sim_get_thermal_tick(self.session, self.i2_avg):
            i2_u, i2_v, i2_w = self.i2_avg[0], self.i2_avg[1], self.i2_avg[2]
            i2_avg_fet = (i2_u + i2_v + i2_w) / 3.0
            p_cond = i2_avg_fet * 0.00335 / 8.0

            V_BUS = 25.2; T_RISE = 15e-9; T_FALL = 10e-9; F_SW = 48000.0
            i_rms_fet = (i2_avg_fet) ** 0.5
            p_sw = (V_BUS * 0.9 * i_rms_fet * (T_RISE + T_FALL) * F_SW) / 4.0
            p_fet = p_cond + p_sw

            p_shunt_u = i2_u * 0.002
            p_shunt_v = i2_v * 0.002

            powers = {f"Q{i}": p_fet for i in range(1, 13)}
            powers["R22"] = p_shunt_u; powers["R23"] = p_shunt_v
            powers.update({"U1": 0.020, "U2": 0.165, "U3": 0.183, "U4": 0.020, "U5": 0.158, "U6": 0.150, "U9": 0.107})

            self.last_T_fields = self.thermal.step(powers)
            self.max_T_fet = float(self.last_T_fields[0].max())
            self.thermal_version += 1

        current_tab = self.tabs.currentIndex()
        if current_tab == 0:
            self._redraw()
        elif current_tab == 1:
            self._update_thermal_view()
        elif current_tab == 2:
            last = self.ring[(self.total_cycles - 1) % RING_SIZE]
            i_u = last[COL_IA_TRUE]
            i_v = -0.5 * i_u + 0.8660254 * last[COL_IB_TRUE]
            i_w = -i_u - i_v
            i_peak = max(abs(i_u), abs(i_v), abs(i_w))

            current_slider_val = int(i_peak * 10)
            if abs(self.gnd_solver_app.slider.value() - current_slider_val) > 5:
                self.gnd_solver_app.slider.setValue(current_slider_val)

    def _redraw(self):
        if self.total_cycles < 2: return
        N = min(self.window_len, self.total_cycles)
        start = (self.total_cycles - N) % RING_SIZE
        if start + N <= RING_SIZE:
            data = self.ring[start:start+N]
        else:
            data = np.vstack([self.ring[start:], self.ring[:start+N-RING_SIZE]])

        x = data[:, COL_TIME] - data[0, COL_TIME]
        self.p_pwm.setXRange(0, N, padding=0)
        self.c_ccr1.setData(x, data[:, COL_CCR1] / 1770.0)
        self.c_ccr2.setData(x, data[:, COL_CCR2] / 1770.0)
        self.c_ccr3.setData(x, data[:, COL_CCR3] / 1770.0)

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
        i_beta = -(i_u + 2.0 * i_v) * 0.577350269

        omega_e = data[:, COL_OMEGA_E]
        advanced_angle = data[:, COL_THETA_E] + omega_e * 40e-6
        cos_a = np.cos(advanced_angle)
        sin_a = np.sin(advanced_angle)
        id_calc =  i_alpha * cos_a + i_beta * sin_a
        iq_calc = -i_alpha * sin_a + i_beta * cos_a

        self.c_id.setData(x, id_calc)
        self.c_iq.setData(x, iq_calc)

        # Correct instantaneous current magnitude for the ground bounce
        i_u_true = data[:, COL_IA_TRUE]
        i_v_true = -0.5 * i_u_true + 0.8660254 * data[:, COL_IB_TRUE]
        i_w_true = -i_u_true - i_v_true
        i_mag = np.sqrt(i_u_true**2 + i_v_true**2 + i_w_true**2)

        v_bounce_mv = i_mag * abs(self.r_adc - self.r_mcu) * 1000.0
        self.c_gnd.setData(x, v_bounce_mv)
        self.c_vbus.setData(x, data[:, COL_VBUS])

        if self.cb_follow.isChecked():
            i_max = max(2.0, 1.2 * max(np.abs(data[:, COL_IA_TRUE]).max(), np.abs(data[:, COL_IB_TRUE]).max()))
            if np.isfinite(i_max): self.p_i.setYRange(-i_max, i_max, padding=0)

            om_max = max(50.0, 1.2 * max(np.abs(data[:, COL_OMEGA_T]).max(), np.abs(data[:, COL_OMEGA_E]).max()))
            if np.isfinite(om_max): self.p_om.setYRange(-om_max, om_max, padding=0)

            idq_max = max(5.0, 1.2 * max(np.abs(id_calc).max(), np.abs(iq_calc).max()))
            if np.isfinite(idq_max): self.p_idq.setYRange(-idq_max, idq_max, padding=0)

            gnd_max = max(5.0, 1.2 * np.abs(v_bounce_mv).max())
            if np.isfinite(gnd_max): self.p_gnd.setYRange(0, gnd_max, padding=0)

        last = data[-1]
        st_name = STATE_NAMES[int(last[COL_STATE])]
        self.lbl_state.setText(
            f"STATE: {st_name:6s} | Ω_true: {last[COL_OMEGA_T]:+7.0f} | Ω_est: {last[COL_OMEGA_E]:+7.0f} | "
            f"θ_t: {last[COL_THETA_T]:4.2f} | θ_e: {last[COL_THETA_E]:4.2f} | Iq: {iq_calc[-1]:.1f}A | Cycle: {int(last[COL_TIME])}"
        )

    def _update_thermal_view(self):
        if self.thermal_version == self._last_thermal_version: return
        self._last_thermal_version = self.thermal_version
        T_fields = self.last_T_fields
        t_min = min(float(T.min()) for T in T_fields)
        t_max = max(float(T.max()) for T in T_fields)
        if t_max - t_min < 0.5: t_max = t_min + 0.5

        for i, T in enumerate(T_fields):
            self._thermal_imgs[i].setImage(np.ascontiguousarray(T.T), autoLevels=False, levels=(t_min, t_max), lut=self._thermal_lut)
            self._thermal_imgs[i].setRect(self._thermal_rect)
        self._thermal_cbar.setLevels((t_min, t_max))
        self.lbl_thermal.setText(f"T_FET_max: {self.max_T_fet:6.1f} °C | T_PCB: {t_min:5.1f} .. {t_max:5.1f} °C")

    def on_stall_toggle(self):
        stall_val = 1 if self.cb_stall.isChecked() else 0
        self.lib.sim_set_stall(self.session, ctypes.c_int(stall_val))

    def toggle_play(self):
        self.playing = self.btn_play.isChecked()
        if self.playing:
            self.timer.start()
        else:
            self.timer.stop()

    def on_throttle(self):
        val = self.sl_throttle.value() / 1000.0
        self.lbl_throttle.setText(f"{val:.2f}")
        self.lib.sim_set_throttle(self.session, ctypes.c_float(val))

    def on_speed(self):
        new_val = self.sl_speed.value()
        self.lbl_speed.setText(f"{new_val}/f")
        if new_val > self.buffer.shape[0]:
            self.cycles_per_frame = new_val
            self.buffer = np.zeros((self.cycles_per_frame, N_COLS), dtype=np.float32)
            self.lib.sim_destroy(self.session)
            self.session = self.lib.sim_create(42, self.buffer.ctypes.data_as(ctypes.POINTER(ctypes.c_float)))
            self.lib.sim_set_throttle(self.session, ctypes.c_float(self.sl_throttle.value() / 1000.0))
        else:
            self.cycles_per_frame = new_val

    def on_window(self):
        self.window_len = self.sl_window.value()
        self.lbl_window.setText(f"{self.window_len}c")

    def reset_sim(self):
        self.timer.stop()
        self.btn_play.setChecked(False)
        self.btn_play.setText("▶ Play")
        self.playing = False
        self.cb_stall.setChecked(False)
        self.ring.fill(0)
        self.total_cycles = 0
        self.lib.sim_reset(self.session, 42)

def main():
    so_path = base / "foc_sim.so"
    geom_path = hil_dir / "pcb_geometry.json"

    if not so_path.exists() or not geom_path.exists():
        print("[!] Missing foc_sim.so or pcb_geometry.json")
        sys.exit(1)

    with open(geom_path) as f:
        geom = json.load(f)

    app = QApplication(sys.argv)
    app.setStyleSheet(STYLESHEET)

    win = FOCScope(geom, so_path)
    win.move(100, 100)  # Force it to a specific X/Y position on screen
    win.show()

    # Force WSLg to bring it to the front
    win.raise_()
    win.activateWindow()

    sys.exit(app.exec())

if __name__ == "__main__":
    main()