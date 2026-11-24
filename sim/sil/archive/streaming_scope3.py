"""
streamingscope3.py — Combined FOC SIL scope + PCB thermal monitor.

Assumes folder structure:
  sim/
    ├── sil/  (this script, foc_sim.so, plant.py)
    └── hil/  (unified_esc_dashboard.py, pcb_geometry.json)
"""
import sys
import json
import ctypes
import numpy as np
from pathlib import Path

# ── Path Setup: Bridge SIL to HIL ───────────────────────────────────────
base = Path(__file__).parent.resolve()
hil_dir = base.parent / "hil"       # sibling folder

if hil_dir.exists():
    sys.path.insert(0, str(hil_dir)) # allow importing from hil
else:
    print(f"[!] Warning: {hil_dir} not found. Imports may fail.")

try:
    from unified_esc_dashboard import TransientSolver, LAYERS
except ImportError as e:
    print(f"[!] Failed to import from {hil_dir}: {e}")
    sys.exit(1)

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QSlider, QLabel, QPushButton, QCheckBox, QTabWidget,
)
from PyQt6.QtCore import Qt, QRectF, QTimer
import pyqtgraph as pg
from pyqtgraph import ColorBarItem

from sim.sil.archive.plant import PMSPlant

# ── Override pyqtgraph config that unified_esc_dashboard set at import ──
pg.setConfigOptions(
    antialias=True,
    useOpenGL=True,
    background='#0a0a0a',
    foreground='#ddd',
)

# ── Ring-buffer column indices ──────────────────────────────────────────
COL_TIME     = 0
COL_CCR1     = 1
COL_CCR2     = 2
COL_CCR3     = 3
COL_IA_TRUE  = 4
COL_IB_TRUE  = 5
COL_IU_FW    = 6
COL_IV_FW    = 7
COL_THETA_T  = 8
COL_THETA_E  = 9
COL_OMEGA_T  = 10
COL_OMEGA_E  = 11
COL_STATE    = 12
COL_THROTTLE = 13
N_COLS = 14

RING_SIZE = 40000

# Enum order from foc_core.h
STATE_NAMES = ["OFF", "ALIGN", "BLEND", "CLOSED", "KICK"]

# ── SIL timing ──────────────────────────────────────────────────────────
TIM2_TICKS_PER_CYCLE  = 3541      # 20.833 µs @ 170 MHz → 48 kHz
THERMAL_PERIOD_CYCLES = 24000     # 0.5 s sim time = TransientSolver.dt


# ═══════════════════════════════════════════════════════════════════════════
#  ctypes bindings for foc_sim.so
# ═══════════════════════════════════════════════════════════════════════════
def load_firmware(so_path):
    lib = ctypes.CDLL(str(so_path))
    lib.foc_set_adc_offsets.argtypes    = [ctypes.c_float, ctypes.c_float]
    lib.sil_adc1_set_jdr1.argtypes      = [ctypes.c_uint32]
    lib.sil_adc2_set_jdr1.argtypes      = [ctypes.c_uint32]
    lib.sil_tim1_get_ccr1.restype       = ctypes.c_uint32
    lib.sil_tim1_get_ccr2.restype       = ctypes.c_uint32
    lib.sil_tim1_get_ccr3.restype       = ctypes.c_uint32
    lib.sil_tim2_advance.argtypes       = [ctypes.c_uint32]
    lib.sil_dshot_get_throttle.restype  = ctypes.c_float
    lib.sil_dshot_set_throttle.argtypes = [ctypes.c_float]
    lib.sil_dshot_set_armed.argtypes    = [ctypes.c_uint8]
    lib.foc_get_state.restype           = ctypes.c_int
    lib.foc_get_estimated_speed.restype = ctypes.c_float
    lib.foc_get_estimated_angle.restype = ctypes.c_float
    lib.foc_get_phase_u_current.restype = ctypes.c_float
    lib.foc_get_phase_v_current.restype = ctypes.c_float
    return lib


# ═══════════════════════════════════════════════════════════════════════════
#  Session — owns firmware + plant + thermal solver, steps them in lockstep
# ═══════════════════════════════════════════════════════════════════════════
class FOCSession:
    """Passive thermal monitor. Motor Rs is NOT fed back into plant."""

    RDS_ON_THERMAL = 0.00335    # per-FET, from plant.py

    def __init__(self, geom, so_path):
        self._plant_kwargs = dict(
            RS=0.110, LS=30e-6, LAMBDA=0.00041, POLE_PAIRS=7,
            INERTIA=5e-6, V_BUS=25.2, BETA_SIGN=+1, DAMPING=2e-5,
        )
        self.plant = PMSPlant(**self._plant_kwargs)
        self.lib   = load_firmware(so_path)

        # Passive IC dissipation (constant across FOC states)
        initial_powers = {
            "U1": 0.020, "U2": 0.165, "U3": 0.183, "U4": 0.020,
            "U5": 0.158, "U6": 0.150, "U9": 0.107,
            "R22": 0.0, "R23": 0.0,
        }
        for i in range(1, 13):
            initial_powers[f"Q{i}"] = 0.0

        # One-time 2–5 s sparse factorization — do it here, not per tick
        print("[*] Factorizing thermal matrix… (2–5 s)")
        self.thermal = TransientSolver(
            geom, initial_powers, h_conv=15.0, h_sink=150.0,
        )
        print("[+] Thermal solver ready.")

        # Independent phase accumulators for accurate shunt heating
        self._i2_sum_u = 0.0
        self._i2_sum_v = 0.0
        self._i2_sum_w = 0.0
        self._i2_n     = 0

        self.cycle           = 0
        self.last_T_fields   = None
        self.max_T_fet       = 25.0
        self.thermal_version = 0

        self._firmware_init()

    # ── public ──────────────────────────────────────────────────────────

    def set_throttle(self, throttle):
        self.lib.sil_dshot_set_throttle(float(throttle))

    def reset(self):
        self.plant.__init__(**self._plant_kwargs)
        self.cycle   = 0
        self._i2_sum_u = 0.0
        self._i2_sum_v = 0.0
        self._i2_sum_w = 0.0
        self._i2_n     = 0
        self._firmware_init()

    def step(self):
        lib, plant = self.lib, self.plant

        ccr1 = lib.sil_tim1_get_ccr1()
        ccr2 = lib.sil_tim1_get_ccr2()
        ccr3 = lib.sil_tim1_get_ccr3()

        plant.step(ccr1, ccr2, ccr3)

        cnt_u, cnt_v = plant.adc_counts()
        lib.sil_adc1_set_jdr1(cnt_u)
        lib.sil_adc2_set_jdr1(cnt_v)
        lib.sil_tim2_advance(TIM2_TICKS_PER_CYCLE)
        lib.foc_loop()

        # Accumulate I²R from TRUE plant phase currents
        i_u, i_v = plant.get_adc_currents(raw=True)
        i_w      = -i_u - i_v

        # Track each phase individually for accurate shunt heating
        self._i2_sum_u += i_u * i_u
        self._i2_sum_v += i_v * i_v
        self._i2_sum_w += i_w * i_w
        self._i2_n     += 1

        # Thermal tick every 0.5 s sim time
        if self.cycle > 0 and self.cycle % THERMAL_PERIOD_CYCLES == 0:
            self._thermal_tick()

        self.cycle += 1

    # ── internals ───────────────────────────────────────────────────────

    def _firmware_init(self):
        self.lib.foc_set_adc_offsets(2048.0, 2048.0)
        self.lib.foc_setup()
        self.lib.sil_dshot_set_armed(1)

    def _thermal_tick(self):
        if self._i2_n == 0:
            return

        # Average the I² for each specific phase over the 0.5s window
        i2_avg_u = self._i2_sum_u / self._i2_n
        i2_avg_v = self._i2_sum_v / self._i2_n
        i2_avg_w = self._i2_sum_w / self._i2_n

        # Average I² across the 3 phases to find the heat per FET.
        # Because we average over 0.5s, the HS/LS switching is already baked in.
        i2_avg_fet = (i2_avg_u + i2_avg_v + i2_avg_w) / 3.0

        # PHYSICAL REALITY: 12 FETs total. 4 per phase leg.
        # P_fet = (I_rms^2 * Rds_on/2) / 4 = I_rms^2 * Rds_on / 8
        p_fet = i2_avg_fet * self.RDS_ON_THERMAL / 8.0

        # Shunts are phase-specific: R22 takes Phase U, R23 takes Phase V
        p_shunt_u = i2_avg_u * 0.002
        p_shunt_v = i2_avg_v * 0.002

        powers = {f"Q{i}": p_fet for i in range(1, 13)}
        powers["R22"] = p_shunt_u
        powers["R23"] = p_shunt_v
        powers.update({"U1": 0.020, "U2": 0.165, "U3": 0.183, "U4": 0.020,
                       "U5": 0.158, "U6": 0.150, "U9": 0.107})

        self.last_T_fields = self.thermal.step(powers)
        self.max_T_fet = float(self.last_T_fields[0].max())  # F_Cu layer
        self.thermal_version += 1

        # Reset accumulators for the next 0.5s window
        self._i2_sum_u = 0.0
        self._i2_sum_v = 0.0
        self._i2_sum_w = 0.0
        self._i2_n = 0


# ═══════════════════════════════════════════════════════════════════════════
#  FOCScope main window
# ═══════════════════════════════════════════════════════════════════════════
class FOCScope(QMainWindow):
    def __init__(self, session):
        super().__init__()
        self.session = session

        self.setWindowTitle("FOC SIL — live scope + PCB thermal")
        self.resize(1400, 1200)

        self.ring = np.zeros((RING_SIZE, N_COLS), dtype=np.float32)
        self.total_cycles = 0

        self.playing          = False
        self.cycles_per_frame = 1440    # ≈ 1× real time at 30 ms tick
        self.window_len       = 800

        self._last_thermal_version = -1

        cw = QWidget()
        self.setCentralWidget(cw)
        layout = QVBoxLayout(cw)

        # ── Control strip ─────────────────────────────────────────────
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
        self.lbl_throttle.setStyleSheet(
            "font-weight:bold; color:#ffb733; font-size:14px; min-width:50px;"
        )
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

        self.cb_follow = QCheckBox("Autoscale")
        self.cb_follow.setChecked(True)
        ctrl.addWidget(self.cb_follow)

        layout.addLayout(ctrl)

        # ── Scope state label ─────────────────────────────────────────
        self.lbl_state = QLabel(
            "STATE: OFF  •  Ω_est:  0  •  Ω_true: 0  •  throttle: 0.00"
        )
        self.lbl_state.setStyleSheet(
            "font-family: monospace; font-size: 13px; color: #00f0ff; "
            "background: #151515; padding: 6px; border: 1px solid #333;"
        )

        # ── Scope plots ───────────────────────────────────────────────
        self.graphics = pg.GraphicsLayoutWidget()
        self._build_scope_plots()

        # ── Tabs: Scope | PCB Thermal ─────────────────────────────────
        scope_widget = QWidget()
        scope_layout = QVBoxLayout(scope_widget)
        scope_layout.setContentsMargins(0, 0, 0, 0)
        scope_layout.addWidget(self.lbl_state)
        scope_layout.addWidget(self.graphics)

        self.tabs = QTabWidget()
        self.tabs.addTab(scope_widget, "Scope")
        self.tabs.addTab(self._build_thermal_tab(), "PCB Thermal")
        layout.addWidget(self.tabs)

        # ── Timer ─────────────────────────────────────────────────────
        self.timer = QTimer()
        self.timer.setInterval(30)
        self.timer.timeout.connect(self.on_tick)

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

        for p in (self.p_i, self.p_theta, self.p_om, self.p_idq):
            p.setXLink(self.p_pwm)

    def _build_thermal_tab(self):
        o = self.session.thermal.geom["outline"]
        self._thermal_rect = QRectF(o["x_mm"], o["y_mm"], o["w_mm"], o["h_mm"])

        w = QWidget()
        v = QVBoxLayout(w)

        self.lbl_thermal = QLabel(
            "T_FET: 25.0 °C   |   waiting for first thermal tick…"
        )
        self.lbl_thermal.setStyleSheet(
            "font-family: monospace; font-size: 13px; color: #ffb733; "
            "background: #151515; padding: 6px; border: 1px solid #333;"
        )
        v.addWidget(self.lbl_thermal)

        g = pg.GraphicsLayoutWidget()
        v.addWidget(g)

        self._thermal_cmap = pg.colormap.get('turbo')
        self._thermal_lut  = self._thermal_cmap.getLookupTable(nPts=256)

        self._thermal_imgs = {}
        for row in range(2):
            for col in range(3):
                p = g.addPlot(row=row, col=col)
                p.setAspectLocked(True)
                p.invertY(True)
                p.hideAxis('left')
                p.hideAxis('bottom')
                p.setRange(self._thermal_rect, padding=0.05)
                L_idx = row * 3 + col
                p.setTitle(LAYERS[L_idx])
                img = pg.ImageItem()
                p.addItem(img)
                self._thermal_imgs[L_idx] = img

        self._thermal_cbar = ColorBarItem(
            values=(25.0, 95.0), colorMap=self._thermal_cmap
        )
        self._thermal_cbar.setImageItem(
            [self._thermal_imgs[i] for i in range(6)]
        )
        g.addItem(self._thermal_cbar, row=0, col=3, rowspan=2)
        return w

    def _update_thermal_view(self):
        if self.session.thermal_version == self._last_thermal_version:
            return
        T_fields = self.session.last_T_fields
        if T_fields is None:
            return
        self._last_thermal_version = self.session.thermal_version

        t_min = min(float(T.min()) for T in T_fields)
        t_max = max(float(T.max()) for T in T_fields)
        if t_max - t_min < 0.5:
            t_max = t_min + 0.5

        for i, T in enumerate(T_fields):
            self._thermal_imgs[i].setImage(
                np.ascontiguousarray(T.T),
                autoLevels=False, levels=(t_min, t_max), lut=self._thermal_lut,
            )
            self._thermal_imgs[i].setRect(self._thermal_rect)

        self._thermal_cbar.setLevels((t_min, t_max))
        self.lbl_thermal.setText(
            f"T_FET_max: {self.session.max_T_fet:6.1f} °C   |   "
            f"T_PCB: {t_min:5.1f} .. {t_max:5.1f} °C   |   "
            f"cycle: {self.session.cycle}"
        )

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
        self.session.set_throttle(val)

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
        self.session.reset()
        self._redraw()

    def on_tick(self):
        lib = self.session.lib
        plant = self.session.plant

        # 1. Run the physics and firmware (MUST happen regardless of tab)
        for _ in range(self.cycles_per_frame):
            self.session.step()

            row = np.array([
                self.total_cycles,
                lib.sil_tim1_get_ccr1(),
                lib.sil_tim1_get_ccr2(),
                lib.sil_tim1_get_ccr3(),
                plant.i_alpha,
                plant.i_beta,
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

        # 2. Only redraw the tab the user is actually looking at
        current_tab = self.tabs.currentIndex()

        if current_tab == 0:  # Scope Tab
            self._redraw()
        elif current_tab == 1:  # PCB Thermal Tab
            self._update_thermal_view()

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
            end   = self.total_cycles % RING_SIZE
            if start < end:
                data = self.ring[start:end]
            else:
                data = np.vstack([self.ring[start:], self.ring[:end]])

        data = np.nan_to_num(data, nan=0.0, posinf=0.0, neginf=0.0)
        x    = data[:, COL_TIME] - data[0, COL_TIME]
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
        i_beta  = (i_u + 2.0 * i_v) * 0.577350269

        omega_e        = data[:, COL_OMEGA_E]
        advanced_angle = data[:, COL_THETA_E] + omega_e * 40e-6

        cos_a = np.cos(advanced_angle)
        sin_a = np.sin(advanced_angle)
        id_calc =  i_alpha * cos_a + i_beta * sin_a
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
                np.abs(iq_calc).max(),
            ))
            self.p_idq.setYRange(-idq_max, idq_max, padding=0)

            pwm_min = min(
                data[:, COL_CCR1].min(),
                data[:, COL_CCR2].min(),
                data[:, COL_CCR3].min(),
            ) * ccr_scale
            pwm_max = max(
                data[:, COL_CCR1].max(),
                data[:, COL_CCR2].max(),
                data[:, COL_CCR3].max(),
            ) * ccr_scale
            self.p_pwm.setYRange(
                max(-0.05, pwm_min - 0.05),
                min(1.05, pwm_max + 0.05),
                padding=0,
            )

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


# ═══════════════════════════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════════════════════════
def main():
    so_path = base / "foc_sim.so"
    if not so_path.exists():
        print(f"[!] {so_path} not found — run `make` first")
        sys.exit(1)

    # Look for pcb_geometry.json in the HIL folder
    geom_path = hil_dir / "pcb_geometry.json"
    if not geom_path.exists():
        print(f"[!] {geom_path} not found.")
        print("    Run `python pcb_extract_geometry.py` inside the hil folder first.")
        sys.exit(1)

    with open(geom_path) as f:
        geom = json.load(f)

    session = FOCSession(geom, so_path)

    app = QApplication(sys.argv)
    win = FOCScope(session)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()