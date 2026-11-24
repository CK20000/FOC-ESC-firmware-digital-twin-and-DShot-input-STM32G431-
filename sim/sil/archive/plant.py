import math
import random
from collections import deque

class PMSPlant:
    def __init__(self,
                 RS=0.110,             # Default is now hot
                 LS=30e-6,
                 LAMBDA=0.00041,
                 POLE_PAIRS=7,
                 INERTIA=5e-6,
                 V_BUS=25.2,
                 DT_SUB=5e-6,
                 BETA_SIGN=+1,
                 DAMPING=2e-5,
                 PROP_LOAD=1.2e-7,
                 seed=42):
        self.RS = RS
        self.LS = LS
        self.LAMBDA = LAMBDA
        self.POLE_PAIRS = POLE_PAIRS
        self.INERTIA = INERTIA
        self.V_BUS_OC = V_BUS
        self.DT_SUB = DT_SUB
        self.BETA_SIGN = BETA_SIGN
        self.DAMPING = DAMPING
        self.PROP_LOAD = PROP_LOAD

        random.seed(seed)

        self.R_BATT = 0.020
        self.R_DS_ON = 0.00335
        self.T_DEAD = 200e-9
        self.F_PWM = 48000.0
        self.T_PWM = 1.0 / self.F_PWM

        self.off_u = random.gauss(0, 15)
        self.off_v = random.gauss(0, 15)
        self.gain_u = 1.0 + random.gauss(0, 0.01)
        self.gain_v = 1.0 + random.gauss(0, 0.01)

        self.AMPS_PER_COUNT = (3.3 / 4095.0) / (0.002 * 20.0)
        self.MAX_READABLE_AMPS = 41.25
        self.SAMPLE_JITTER = 0.05 # ADC trigger position error (fraction of ripple)

        self.theta = random.uniform(0, 2.0 * math.pi)
        self.i_alpha = 0.0
        self.i_beta = 0.0
        self.omega = 0.0
        self.load_torque = 0.0
        self.v_bus_sag = self.V_BUS_OC

        self.i_u_filt = 0.0
        self.i_v_filt = 0.0
        self.last_ccr = (885, 885, 885)

        # FIXED: Leftover 2125 changed to 885 for 48kHz neutral
        self.pwm_pipe = deque([(885, 885, 885)] * 2, maxlen=2)

    def ccr_to_voltages(self, ccr1, ccr2, ccr3, arr=1770.0):
        da = ccr1 / arr
        db = ccr2 / arr
        dc = ccr3 / arr

        va = (da - 0.5) * self.v_bus_sag
        vb = (db - 0.5) * self.v_bus_sag
        vc = (dc - 0.5) * self.v_bus_sag

        i_u, i_v = self.get_adc_currents(raw=True)
        i_w = -i_u - i_v

        v_dead_scale = (self.T_DEAD / self.T_PWM) * self.v_bus_sag
        def soft_sign(x): return x / math.sqrt(x*x + 0.1)

        va -= v_dead_scale * soft_sign(i_u)
        vb -= v_dead_scale * soft_sign(i_v)
        vc -= v_dead_scale * soft_sign(i_w)

        va -= i_u * 2.0 * self.R_DS_ON
        vb -= i_v * 2.0 * self.R_DS_ON
        vc -= i_w * 2.0 * self.R_DS_ON

        vcom = (va + vb + vc) / 3.0
        return va - vcom, vb - vcom, vc - vcom

    def step(self, ccr1, ccr2, ccr3, dt=20.833e-6):
        self.pwm_pipe.append((ccr1, ccr2, ccr3))
        delayed_ccr1, delayed_ccr2, delayed_ccr3 = self.pwm_pipe[0]
        self.last_ccr = (delayed_ccr1, delayed_ccr2, delayed_ccr3)

        i_mag = math.sqrt(self.i_alpha**2 + self.i_beta**2)
        self.v_bus_sag = self.V_BUS_OC - (i_mag * 0.707 * self.R_BATT)

        va, vb, vc = self.ccr_to_voltages(delayed_ccr1, delayed_ccr2, delayed_ccr3)
        v_alpha = va
        v_beta = (va + 2.0 * vb) / math.sqrt(3.0)

        n_sub = max(1, int(round(dt / self.DT_SUB)))
        dts = dt / n_sub

        self.load_torque = random.uniform(-0.002, 0.002)

        for _ in range(n_sub):
            e_alpha = -self.LAMBDA * self.omega * math.sin(self.theta)
            e_beta = self.LAMBDA * self.omega * math.cos(self.theta)

            di_alpha = (v_alpha - self.RS * self.i_alpha - e_alpha) / self.LS
            di_beta = (v_beta - self.RS * self.i_beta - e_beta) / self.LS
            self.i_alpha += di_alpha * dts
            self.i_beta += di_beta * dts

            i_d = self.i_alpha * math.cos(self.theta) + self.i_beta * math.sin(self.theta)
            i_q = -self.i_alpha * math.sin(self.theta) + self.i_beta * math.cos(self.theta)
            torque = 1.5 * self.POLE_PAIRS * self.LAMBDA * i_q

            omega_mech = self.omega / self.POLE_PAIRS
            prop_torque = self.PROP_LOAD * omega_mech * abs(omega_mech)
            alpha_mech = (torque - self.load_torque - prop_torque
                          - self.DAMPING * omega_mech) / self.INERTIA
            omega_mech += alpha_mech * dts
            self.omega = omega_mech * self.POLE_PAIRS
            self.theta += self.omega * dts
            self.theta %= 2.0 * math.pi

    def get_adc_currents(self, raw=False):
        i_alpha = self.i_alpha
        i_beta_eff = self.i_beta * self.BETA_SIGN
        i_u = i_alpha
        i_v = -0.5 * i_alpha + math.sqrt(3.0) / 2.0 * i_beta_eff

        if raw:
            return i_u, i_v

        alpha = (2.0 * math.pi * 400000.0 * 20.833e-6) / (1.0 + 2.0 * math.pi * 400000.0 * 20.833e-6)
        self.i_u_filt += alpha * (i_u - self.i_u_filt)
        self.i_v_filt += alpha * (i_v - self.i_v_filt)

        return self.i_u_filt, self.i_v_filt

    def adc_counts(self, offset=2048.0):
        i_u, i_v = self.get_adc_currents()

        i_u_clipped = max(-self.MAX_READABLE_AMPS, min(self.MAX_READABLE_AMPS, i_u))
        i_v_clipped = max(-self.MAX_READABLE_AMPS, min(self.MAX_READABLE_AMPS, i_v))

        cnt_u = offset + self.off_u + self.gain_u * (i_u_clipped / self.AMPS_PER_COUNT)
        cnt_v = offset + self.off_v + self.gain_v * (i_v_clipped / self.AMPS_PER_COUNT)

        # --- FIXED: TRUE PHYSICAL RIPPLE ---
        # Ripple only exists if there is a voltage difference between phases.
        d = [c / 1770.0 for c in self.last_ccr]
        spread = max(d) - min(d)
        ripple_pp = (self.v_bus_sag * spread * (1.0 - spread)) / (self.LS * self.F_PWM)

        # Independent ADC trigger jitter for U and V
        jitter_u = ripple_pp * self.SAMPLE_JITTER * random.uniform(-1.0, 1.0) / self.AMPS_PER_COUNT
        jitter_v = ripple_pp * self.SAMPLE_JITTER * random.uniform(-1.0, 1.0) / self.AMPS_PER_COUNT

        emi_scale = 0.05 * abs(i_u) + 0.05 * abs(i_v)
        spike_u = random.uniform(-emi_scale, emi_scale)
        spike_v = random.uniform(-emi_scale, emi_scale)

        # FIXED: Use round() instead of int()
        cnt_u = round(cnt_u + jitter_u + spike_u)
        cnt_v = round(cnt_v + jitter_v + spike_v)

        return max(0, min(4095, cnt_u)), max(0, min(4095, cnt_v))

    def true_speed_rpm(self):
        return self.omega / self.POLE_PAIRS * 60.0 / (2.0 * math.pi)