"""
plant.py — Real-world PMSM physics for SIL testing.
Includes: Bus sag, MOSFET Rds(on), Dead-Time distortion, AD8418A bandwidth,
ADC switching noise, and high-frequency PWM current ripple.
"""

import math
import random

class PMSPlant:
    def __init__(self,
                 RS=0.090,
                 LS=30e-6,
                 LAMBDA=0.00041,
                 POLE_PAIRS=7,
                 INERTIA=5e-6,
                 V_BUS=25.2,           # Open-circuit battery voltage (6S)
                 DT_SUB=5e-6,
                 BETA_SIGN=+1,
                 DAMPING=2e-5,
                 PROP_LOAD=1e-8):
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

        # --- HARDWARE PARAMETERS ---
        self.R_BATT = 0.020       # 6S LiPo internal resistance (20 mOhm)
        self.R_DS_ON = 0.00335    # AOD4184A Rds(on) per FET (3.35 mOhm)
        self.T_DEAD = 200e-9      # EG2134 dead-time (200 ns)
        self.F_PWM = 40000.0      # 40 kHz PWM frequency
        self.T_PWM = 1.0 / self.F_PWM

        # --- OBSERVER & SENSOR PARAMETERS ---
        self.FC_AMP = 400000.0    # AD8418A bandwidth (~400 kHz)
        self.AMPS_PER_COUNT = (3.3 / 4095.0) / (0.002 * 20.0) # 2mOhm shunt, 20V/V gain

        # --- STATE VARIABLES ---
        self.i_alpha = 0.0
        self.i_beta = 0.0
        self.omega = 0.0
        self.theta = 0.0
        self.load_torque = 0.0
        self.v_bus_sag = self.V_BUS_OC

        # Sensor filters
        self.i_u_filt = 0.0
        self.i_v_filt = 0.0

    def ccr_to_voltages(self, ccr1, ccr2, ccr3, arr=1770.0):
        """Converts duty cycles (CCR) to phase voltages, applying dead-time and Rds(on) distortion."""
        da = ccr1 / arr
        db = ccr2 / arr
        dc = ccr3 / arr

        va = (da - 0.5) * self.v_bus_sag
        vb = (db - 0.5) * self.v_bus_sag
        vc = (dc - 0.5) * self.v_bus_sag

        # Get true phase currents (before filtering)
        i_u, i_v = self.get_adc_currents(raw=True)
        i_w = -i_u - i_v

        # 3. Dead-Time Distortion (SMOOTHED)
        # Using a soft-sign to prevent discrete limit cycles at zero-crossing.
        v_dead_scale = self.T_DEAD / self.T_PWM * self.v_bus_sag

        # Soft sign function (tanh approximation): x / sqrt(x^2 + 0.1)
        # This ensures the dead-time voltage smoothly transitions through zero current.
        def soft_sign(x):
            return x / math.sqrt(x*x + 0.1)

        va -= v_dead_scale * soft_sign(i_u)
        vb -= v_dead_scale * soft_sign(i_v)
        vc -= v_dead_scale * soft_sign(i_w)

        # 4. MOSFET Rds(on) Voltage Drop
        va -= i_u * 2.0 * self.R_DS_ON
        vb -= i_v * 2.0 * self.R_DS_ON
        vc -= i_w * 2.0 * self.R_DS_ON

        # 5. Common Mode Subtraction
        vcom = (va + vb + vc) / 3.0
        return va - vcom, vb - vcom, vc - vcom

    def step(self, ccr1, ccr2, ccr3, dt=20.833e-6):
        # 1. Calculate battery sag based on current magnitude
        i_mag = math.sqrt(self.i_alpha**2 + self.i_beta**2)
        self.v_bus_sag = self.V_BUS_OC - (i_mag * 0.707 * self.R_BATT)

        # 2. Get distorted phase voltages
        va, vb, vc = self.ccr_to_voltages(ccr1, ccr2, ccr3)
        v_alpha = va
        v_beta = (va + 2.0 * vb) / math.sqrt(3.0)

        n_sub = max(1, int(round(dt / self.DT_SUB)))
        dts = dt / n_sub

        # Mechanical Noise
        self.load_torque = random.uniform(-0.002, 0.002)

        for _ in range(n_sub):
            e_alpha = -self.LAMBDA * self.omega * math.sin(self.theta)
            e_beta = self.LAMBDA * self.omega * math.cos(self.theta)

            # The Euler integration naturally produces the PWM current ripple
            # because the voltage is fixed for the duration of the sub-step.
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

        # --- AD8418A OP-AMP BANDWIDTH (First-Order LPF) ---
        # Simulates the slight lag of the current sense amplifier.
        alpha = (2.0 * math.pi * self.FC_AMP * 20.833e-6) / (1.0 + 2.0 * math.pi * self.FC_AMP * 20.833e-6)
        self.i_u_filt += alpha * (i_u - self.i_u_filt)
        self.i_v_filt += alpha * (i_v - self.i_v_filt)

        return self.i_u_filt, self.i_v_filt

    def adc_counts(self, offset=2048.0):
        """Simulates the STM32 ADC sampling with realistic noise and switching spikes."""
        i_u, i_v = self.get_adc_currents()

        cnt_u = offset + i_u / self.AMPS_PER_COUNT
        cnt_v = offset + i_v / self.AMPS_PER_COUNT

        # --- ADC NOISE & SWITCHING SPIKES ---
        # White noise
        noise_u = random.gauss(0, 3.0)
        noise_v = random.gauss(0, 3.0)

        # Switching EMI coupling (proportional to current magnitude)
        emi_scale = 0.05 * abs(i_u) + 0.05 * abs(i_v)
        spike_u = random.uniform(-emi_scale, emi_scale)
        spike_v = random.uniform(-emi_scale, emi_scale)

        cnt_u = int(cnt_u + noise_u + spike_u)
        cnt_v = int(cnt_v + noise_v + spike_v)

        return max(0, min(4095, cnt_u)), max(0, min(4095, cnt_v))

    def true_speed_rpm(self):
        return self.omega / self.POLE_PAIRS * 60.0 / (2.0 * math.pi)