"""foc_session.py — Middleware: owns firmware + plant + thermal solver.

Passive thermal monitor only. Motor Rs is NOT fed back into plant.py or
foc_core.c. Thermal tick fires once per 0.5s of SIMULATED time (matches
TransientSolver.dt = 0.5s).
"""

import ctypes
import sys
import math
import numpy as np
from pathlib import Path

from plant import PMSPlant
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


STATE_NAMES = ["OFF", "ALIGN", "BLEND", "CLOSED", "KICK"]

# 48 kHz loop → 20.833 µs per cycle → 3541 ticks @ 170 MHz
TIM2_TICKS_PER_CYCLE = 3541

# 0.5s of simulated time → matches TransientSolver.dt
THERMAL_PERIOD_CYCLES = 24000


def load_firmware(so_path):
    """Same signature binding as run_foc.load_firmware — duplicated so we
    don't depend on run_foc.py."""
    lib = ctypes.CDLL(str(so_path))
    lib.foc_set_adc_offsets.argtypes = [ctypes.c_float, ctypes.c_float]
    lib.sil_adc1_set_jdr1.argtypes   = [ctypes.c_uint32]
    lib.sil_adc2_set_jdr1.argtypes   = [ctypes.c_uint32]
    lib.sil_tim1_get_ccr1.restype    = ctypes.c_uint32
    lib.sil_tim1_get_ccr2.restype    = ctypes.c_uint32
    lib.sil_tim1_get_ccr3.restype    = ctypes.c_uint32
    lib.sil_tim2_advance.argtypes    = [ctypes.c_uint32]
    lib.sil_dshot_get_throttle.restype = ctypes.c_float
    lib.sil_dshot_set_throttle.argtypes = [ctypes.c_float]
    lib.sil_dshot_set_armed.argtypes    = [ctypes.c_uint8]
    lib.foc_get_state.restype             = ctypes.c_int
    lib.foc_get_estimated_speed.restype   = ctypes.c_float
    lib.foc_get_estimated_angle.restype   = ctypes.c_float
    lib.foc_get_phase_u_current.restype   = ctypes.c_float
    lib.foc_get_phase_v_current.restype   = ctypes.c_float
    return lib


class FOCSession:
    """Steps firmware + plant + thermal as a single unit.

    Headless usage:
        session = FOCSession(geom, "foc_sim.so")
        session.set_throttle(0.5)
        for _ in range(480000):           # 10 s sim
            st = session.step()
            if session.cycle % 24000 == 0:
                print(session.max_T_fet)
    """

    # Per-FET thermal power scale (matches dashboard's formula):
    #   P_per_fet = I_rms² × RDS_ON_THERMAL × 2 / 12
    RDS_ON_THERMAL = 0.00335  # from plant.py (per-FET, real value)

    def __init__(self, geom, so_path, plant_kwargs=None):
        self._plant_kwargs = plant_kwargs or dict(
            RS=0.110, LS=30e-6, LAMBDA=0.00041, POLE_PAIRS=7,
            INERTIA=5e-6, V_BUS=25.2, BETA_SIGN=+1, DAMPING=2e-5,
        )
        self.plant = PMSPlant(**self._plant_kwargs)
        self.lib   = load_firmware(so_path)

        # Passive IC dissipation (constant between FOC states)
        initial_powers = {
            "U1": 0.020, "U2": 0.165, "U3": 0.183, "U4": 0.020,
            "U5": 0.158, "U6": 0.150, "U9": 0.107,
            "R22": 0.0, "R23": 0.0,
        }
        for i in range(1, 13):
            initial_powers[f"Q{i}"] = 0.0

        # One-time 2-5 s factorization — do it here so startup is honest
        self.thermal = TransientSolver(
            geom, initial_powers, h_conv=15.0, h_sink=150.0
        )

        # Accumulator (averages I²R over THERMAL_PERIOD_CYCLES)
        self._i2_sum = 0.0
        self._i2_n   = 0

        # Public state
        self.cycle       = 0
        self.last_T_fields = None
        self.max_T_fet   = 25.0

        self._firmware_init()

    # ---------- public API ----------

    def set_throttle(self, throttle):
        self.lib.sil_dshot_set_throttle(float(throttle))

    def reset(self):
        """Reset firmware + plant. Thermal is kept (it's a PCB property)."""
        self.plant.__init__(**self._plant_kwargs)
        self.cycle = 0
        self._i2_sum = 0.0
        self._i2_n   = 0
        self._firmware_init()

    def step(self):
        """Advance exactly one FOC cycle. Returns a state dict for the UI."""
        lib, plant = self.lib, self.plant

        # 1. Read PWM that was latched last cycle
        ccr1 = lib.sil_tim1_get_ccr1()
        ccr2 = lib.sil_tim1_get_ccr2()
        ccr3 = lib.sil_tim1_get_ccr3()

        # 2. Plant electrical + mechanical step
        plant.step(ccr1, ccr2, ccr3)

        # 3. Feed ADC samples + advance timestamp
        cnt_u, cnt_v = plant.adc_counts()
        lib.sil_adc1_set_jdr1(cnt_u)
        lib.sil_adc2_set_jdr1(cnt_v)
        lib.sil_tim2_advance(TIM2_TICKS_PER_CYCLE)

        # 4. Run firmware hot path
        lib.foc_loop()

        # 5. Accumulate I²R using TRUE plant phase currents
        i_u_true, i_v_true = plant.get_adc_currents(raw=True)
        i_w_true = -i_u_true - i_v_true
        self._i2_sum += (i_u_true**2 + i_v_true**2 + i_w_true**2) / 3.0
        self._i2_n   += 1

        # 6. Thermal tick
        if self.cycle > 0 and self.cycle % THERMAL_PERIOD_CYCLES == 0:
            self._thermal_tick()

        self.cycle += 1

        return {
            "cycle":      self.cycle,
            "ccr1":       ccr1, "ccr2": ccr2, "ccr3": ccr3,
            "i_alpha":    plant.i_alpha,
            "i_beta":     plant.i_beta,
            "i_u_fw":     lib.foc_get_phase_u_current(),
            "i_v_fw":     lib.foc_get_phase_v_current(),
            "theta_true": plant.theta,
            "theta_est":  lib.foc_get_estimated_angle(),
            "omega_true": plant.omega,
            "omega_est":  lib.foc_get_estimated_speed(),
            "state":      int(lib.foc_get_state()),
            "throttle":   lib.sil_dshot_get_throttle(),
            "max_T_fet":  self.max_T_fet,
        }

    # ---------- internals ----------

    def _firmware_init(self):
        self.lib.foc_set_adc_offsets(2048.0, 2048.0)
        self.lib.foc_setup()
        self.lib.sil_dshot_set_armed(1)

    def _thermal_tick(self):
        if self._i2_n == 0:
            return
        i2_avg = self._i2_sum / self._i2_n

        # Per-FET power (matches dashboard's convention, scaled to plant's Rds)
        p_fet   = i2_avg * self.RDS_ON_THERMAL * 2.0 / 12.0
        p_shunt = i2_avg * 0.002  # 2 mΩ shunt

        powers = {f"Q{i}": p_fet for i in range(1, 13)}
        powers["R22"] = p_shunt
        powers["R23"] = p_shunt
        powers.update({"U1": 0.020, "U2": 0.165, "U3": 0.183, "U4": 0.020,
                       "U5": 0.158, "U6": 0.150, "U9": 0.107})

        self.last_T_fields = self.thermal.step(powers)
        # FETs live on F_Cu (layer 0)
        self.max_T_fet = float(self.last_T_fields[0].max())

        self._i2_sum = 0.0
        self._i2_n   = 0