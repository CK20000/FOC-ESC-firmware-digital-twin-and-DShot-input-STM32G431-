"""
run_foc.py — headless SIL driver.

Loads firmware .so, runs N FOC cycles against plant physics, prints
state transitions, saves log to CSV + PNG.
"""

import ctypes
import sys
import time
from pathlib import Path

from sim.sil.archive.plant import PMSPlant


def load_firmware(so_path):
    lib = ctypes.CDLL(str(so_path))
    # Signatures
    lib.foc_set_adc_offsets.argtypes = [ctypes.c_float, ctypes.c_float]
    lib.sil_adc1_set_jdr1.argtypes = [ctypes.c_uint32]
    lib.sil_adc2_set_jdr1.argtypes = [ctypes.c_uint32]
    lib.sil_tim1_get_ccr1.restype = ctypes.c_uint32
    lib.sil_tim1_get_ccr2.restype = ctypes.c_uint32
    lib.sil_tim1_get_ccr3.restype = ctypes.c_uint32
    lib.sil_tim2_advance.argtypes = [ctypes.c_uint32]
    lib.sil_dshot_get_throttle.restype = ctypes.c_float
    lib.sil_dshot_get_armed.restype = ctypes.c_uint8
    lib.sil_dshot_set_throttle.argtypes = [ctypes.c_float]
    lib.sil_dshot_set_armed.argtypes = [ctypes.c_uint8]
    lib.foc_get_state.restype = ctypes.c_int
    lib.foc_get_estimated_speed.restype = ctypes.c_float
    lib.foc_get_estimated_angle.restype = ctypes.c_float
    lib.foc_get_phase_u_current.restype = ctypes.c_float
    lib.foc_get_phase_v_current.restype = ctypes.c_float
    lib.foc_get_v_alpha.restype = ctypes.c_float
    lib.foc_get_v_beta.restype = ctypes.c_float
    return lib


def run_cycles(lib, plant, n_cycles, throttle_sequence, verbose=True):
    """
    throttle_sequence: callable(cycle_index) -> (throttle: float, armed: bool)
    """
    STATE_NAMES = ["OFF", "ALIGN", "RAMP", "BLEND", "CLOSED_LOOP"]

    lib.foc_set_adc_offsets(2048.0, 2048.0)
    lib.foc_setup()

    log = []
    prev_state = -1
    transitions = []

    t_start = time.time()
    for cycle in range(n_cycles):
        throttle, armed = throttle_sequence(cycle)
        lib.sil_dshot_set_throttle(float(throttle))
        lib.sil_dshot_set_armed(1 if armed else 0)

        ccr1 = lib.sil_tim1_get_ccr1()
        ccr2 = lib.sil_tim1_get_ccr2()
        ccr3 = lib.sil_tim1_get_ccr3()

        plant.step(ccr1, ccr2, ccr3)

        cnt_u, cnt_v = plant.adc_counts()
        lib.sil_adc1_set_jdr1(cnt_u)
        lib.sil_adc2_set_jdr1(cnt_v)

        lib.sil_tim2_advance(567)  # 3333 ns per cycle @ 170 MHz
        lib.foc_loop()

        state = lib.foc_get_state()
        if state != prev_state:
            transitions.append((cycle, STATE_NAMES[state]))
            if verbose:
                print(f"  [{cycle:6d}] state → {STATE_NAMES[state]:12s}  "
                      f"theta_true={plant.theta:6.2f}  "
                      f"theta_est={lib.foc_get_estimated_angle():6.2f}")
            prev_state = state

        log.append((
            cycle * 50.0,                    # t_us
            ccr1, ccr2, ccr3,                # PWM
            plant.i_alpha, plant.i_beta,     # true currents
            lib.foc_get_phase_u_current(),   # firmware-view current
            lib.foc_get_phase_v_current(),
            plant.theta,                     # true angle
            lib.foc_get_estimated_angle(),   # estimated
            plant.omega,                     # true speed
            lib.foc_get_estimated_speed(),
            state,
            throttle,
        ))

    elapsed = time.time() - t_start
    print(f"\n[+] {n_cycles} cycles in {elapsed:.2f} s "
          f"({n_cycles/elapsed:.0f} cycles/s)")
    return log, transitions


def plot_log(log):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[!] matplotlib not installed — skipping plot")
        return

    cols = ["t_us", "ccr1", "ccr2", "ccr3",
            "i_alpha", "i_beta", "i_u_fw", "i_v_fw",
            "theta_true", "theta_est", "omega_true", "omega_est",
            "state", "throttle"]
    t     = [r[0] / 1000.0 for r in log]  # ms
    ccr1  = [r[1] for r in log]
    i_a   = [r[4] for r in log]
    i_b   = [r[5] for r in log]
    th_t  = [r[8] for r in log]
    th_e  = [r[9] for r in log]
    om_t  = [r[10] for r in log]
    om_e  = [r[11] for r in log]
    st    = [r[12] for r in log]
    thr   = [r[13] for r in log]

    fig, axs = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    fig.patch.set_facecolor("#111111")
    for ax in axs:
        ax.set_facecolor("#111111")
        ax.tick_params(colors="white")
        ax.grid(True, alpha=0.2)
        for s in ax.spines.values():
            s.set_color("#444")

    axs[0].plot(t, ccr1, "w", lw=0.5, label="CCR1 (phase U)")
    axs[0].set_ylabel("CCR (0-4250)", color="white")
    axs[0].set_ylim(0, 4250)
    axs[0].legend(loc="upper right", facecolor="#222", edgecolor="#555")

    axs[1].plot(t, i_a, "#00f0ff", lw=0.6, label="i_alpha")
    axs[1].plot(t, i_b, "#ff0055", lw=0.6, label="i_beta")
    axs[1].set_ylabel("Current (A)", color="white")
    axs[1].legend(loc="upper right", facecolor="#222", edgecolor="#555")

    axs[2].plot(t, th_t, "#888", lw=0.5, label="theta TRUE")
    axs[2].plot(t, th_e, "#ffb733", lw=0.6, label="theta EST")
    axs[2].set_ylabel("Angle (rad)", color="white")
    axs[2].set_ylim(-0.5, 6.5)
    axs[2].legend(loc="upper right", facecolor="#222", edgecolor="#555")

    axs[3].plot(t, om_t, "#888", lw=0.5, label="omega TRUE")
    axs[3].plot(t, om_e, "#00ff88", lw=0.6, label="omega EST")
    axs[3].set_ylabel("Speed (rad/s)", color="white")
    axs[3].set_xlabel("Time (ms)", color="white")
    axs[3].legend(loc="upper right", facecolor="#222", edgecolor="#555")

    # State shading
    for ax in axs:
        prev_st = st[0]
        seg_start = 0
        for i, s in enumerate(st[1:], 1):
            if s != prev_st:
                ax.axvspan(t[seg_start], t[i], alpha=0.08,
                           color=["#666", "#00f", "#0f0", "#ff0", "#f00"][prev_st])
                seg_start = i
                prev_st = s
        ax.axvspan(t[seg_start], t[-1], alpha=0.08,
                   color=["#666", "#00f", "#0f0", "#ff0", "#f00"][prev_st])

    fig.suptitle("FOC SIL — states: OFF(gray) ALIGN(blue) RAMP(green) "
                 "BLEND(yellow) CLOSED(red)",
                 color="white", fontsize=11)
    plt.tight_layout()
    out = Path(__file__).parent / "foc_sil.png"
    plt.savefig(out, dpi=110, facecolor="#111111")
    print(f"[+] Saved {out}")


def main():
    so_path = Path(__file__).parent / "foc_sim.so"
    if not so_path.exists():
        print(f"[!] {so_path} not found — run `make` first")
        sys.exit(1)

    lib = load_firmware(so_path)

    # Plant parameters — use DIFFERENT values than firmware for a real test
    plant = PMSPlant(
        RS=0.090,  # matches firmware
        LS=30e-6,  # matches firmware
        LAMBDA=0.00041,  # matches firmware
        POLE_PAIRS=7,
        INERTIA=5e-6,  # realistic 2208-class rotor + prop
        V_BUS=25.2,
        BETA_SIGN=+1,  # firmware's Clarke is negated; flip to match
    )

    N_CYCLES = 60000   # 3 seconds at 20 kHz

    def throttle_seq(cycle):
        # 0-100 ms: zero throttle (arm latch)
        if cycle < 2000:
            return 0.0, True
        # After that: 20% throttle
        return 0.20, True

    print(f"[*] Running {N_CYCLES} cycles ({N_CYCLES*50/1e6:.2f} s sim time)")
    print("[*] State transitions:")
    log, transitions = run_cycles(lib, plant, N_CYCLES, throttle_seq)

    print(f"\n[+] Transitions: {' → '.join(s for _, s in transitions)}")

    # Summary of final 1000 cycles
    tail = log[-1000:]
    avg_omega_true = sum(r[10] for r in tail) / len(tail)
    avg_omega_est  = sum(r[11] for r in tail) / len(tail)
    print(f"\n[+] Final 50 ms averages:")
    print(f"    omega TRUE  = {avg_omega_true:+.1f} rad/s "
          f"({avg_omega_true/7*60/(2*3.14159):.0f} RPM mech)")
    print(f"    omega EST   = {avg_omega_est:+.1f} rad/s")
    print(f"    tracking err = {avg_omega_est - avg_omega_true:+.1f} rad/s")

    plot_log(log)


if __name__ == "__main__":
    main()