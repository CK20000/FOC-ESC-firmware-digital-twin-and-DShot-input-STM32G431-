import ctypes
import math
import random
import time
from pathlib import Path
from plant import PMSPlant

STATE_NAMES = ["OFF", "ALIGN", "BLEND", "CLOSED", "KICK"]

SNAPSHOT_SCHEDULE = {
    1: [500, 1000],  # ALIGN
    4: [100, 250, 500, 1000, 1500, 2000, 2500, 3000],  # KICK
    2: [1000, 2000, 5000, 8000, 12000],  # BLEND
    3: [500, 1000, 5000, 10000, 20000, 40000, 60000, 80000],  # CLOSED
}


def wrap_pi(a):
    while a > math.pi:   a -= 2 * math.pi
    while a < -math.pi:  a += 2 * math.pi
    return a


def run_one(lib, target_throttle, n_cycles=120000, force_align=False):
    # FIX 1: Pass seed=None to use system time (random every run)
    # FIX 2: Optional force_align to prove the code works from a perfect angle
    seed_val = 42 if force_align else int(time.time() * 1000) % 100000

    plant = PMSPlant(RS=0.110, LS=30e-6, LAMBDA=0.00041, POLE_PAIRS=7,
                     INERTIA=5e-6, V_BUS=25.2, BETA_SIGN=+1, DAMPING=2e-5,
                     seed=seed_val)

    if force_align:
        plant.theta = 1.57

    lib.foc_set_adc_offsets(2048.0, 2048.0)
    lib.foc_setup()
    lib.sil_dshot_set_armed(1)

    # Calibration
    sum_u, sum_v = 0, 0
    for _ in range(4000):
        plant.step(885, 885, 885)
        cnt_u, cnt_v = plant.adc_counts()
        sum_u += cnt_u
        sum_v += cnt_v
        lib.sil_adc1_set_jdr1(cnt_u)
        lib.sil_adc2_set_jdr1(cnt_v)
        lib.sil_tim2_advance(3541)
        lib.foc_loop()
    lib.foc_set_adc_offsets(sum_u / 4000.0, sum_v / 4000.0)

    # Settle at 0% throttle for 20,000 cycles
    lib.sil_dshot_set_throttle(0.0)
    for _ in range(20000):
        ccr1 = lib.sil_tim1_get_ccr1()
        ccr2 = lib.sil_tim1_get_ccr2()
        ccr3 = lib.sil_tim1_get_ccr3()
        plant.step(ccr1, ccr2, ccr3)
        cnt_u, cnt_v = plant.adc_counts()
        lib.sil_adc1_set_jdr1(cnt_u)
        lib.sil_adc2_set_jdr1(cnt_v)
        lib.sil_tim2_advance(3541)
        lib.foc_loop()

    start_theta = plant.theta
    lib.sil_dshot_set_throttle(float(target_throttle))

    snapshots = []
    prev_state = -1
    state_entry_cycle = {}
    captured = {}

    for cycle in range(n_cycles):
        ccr1 = lib.sil_tim1_get_ccr1()
        ccr2 = lib.sil_tim1_get_ccr2()
        ccr3 = lib.sil_tim1_get_ccr3()

        plant.step(ccr1, ccr2, ccr3)
        cnt_u, cnt_v = plant.adc_counts()
        lib.sil_adc1_set_jdr1(cnt_u)
        lib.sil_adc2_set_jdr1(cnt_v)
        lib.sil_tim2_advance(3541)
        lib.foc_loop()

        state = lib.foc_get_state()

        def capture_snapshot(tag):
            w_est = lib.foc_get_estimated_speed()
            ang_e = lib.foc_get_estimated_angle()
            i_u = lib.foc_get_phase_u_current()
            i_v = lib.foc_get_phase_v_current()

            phase_advance = w_est * 40e-6
            advanced_angle = ang_e + phase_advance
            i_alpha = i_u
            i_beta = (i_u + 2.0 * i_v) * 0.577350269

            cos_a = math.cos(advanced_angle)
            sin_a = math.sin(advanced_angle)
            i_d = i_alpha * cos_a + i_beta * sin_a
            i_q = -i_alpha * sin_a + i_beta * cos_a

            return {
                "cycle": cycle, "throttle": target_throttle,
                "state": state, "tag": tag, "w_true": plant.omega,
                "w_est": w_est, "ang_t": plant.theta, "ang_e": ang_e,
                "i_a": plant.i_alpha, "i_b": plant.i_beta, "id": i_d, "iq": i_q,
            }

        if state != prev_state:
            state_entry_cycle[state] = cycle
            captured[state] = set()
            snapshots.append(capture_snapshot(f"{STATE_NAMES[state]}_ENTRY"))
            prev_state = state

        if state in SNAPSHOT_SCHEDULE and state in state_entry_cycle:
            progress = cycle - state_entry_cycle[state]
            for pt in SNAPSHOT_SCHEDULE[state]:
                if progress == pt and pt not in captured[state]:
                    captured[state].add(pt)
                    snapshots.append(capture_snapshot(f"{STATE_NAMES[state]}+{pt}"))

        if cycle > 80000 and state == 3:
            if abs(plant.omega - lib.foc_get_estimated_speed()) < 50:
                break

    return snapshots, start_theta


def main():
    so_path = Path(__file__).parent / "foc_sim.so"
    lib = ctypes.CDLL(str(so_path))

    lib.foc_set_adc_offsets.argtypes = [ctypes.c_float, ctypes.c_float]
    lib.sil_adc1_set_jdr1.argtypes = [ctypes.c_uint32]
    lib.sil_adc2_set_jdr1.argtypes = [ctypes.c_uint32]
    lib.sil_tim1_get_ccr1.restype = ctypes.c_uint32
    lib.sil_tim1_get_ccr2.restype = ctypes.c_uint32
    lib.sil_tim1_get_ccr3.restype = ctypes.c_uint32
    lib.sil_tim2_advance.argtypes = [ctypes.c_uint32]
    lib.sil_dshot_set_throttle.argtypes = [ctypes.c_float]
    lib.sil_dshot_set_armed.argtypes = [ctypes.c_uint8]
    lib.foc_get_estimated_speed.restype = ctypes.c_float
    lib.foc_get_estimated_angle.restype = ctypes.c_float
    lib.foc_get_state.restype = ctypes.c_int
    lib.foc_get_phase_u_current.restype = ctypes.c_float
    lib.foc_get_phase_v_current.restype = ctypes.c_float

    throttles = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00]

    out_lines = []
    out_lines.append("=" * 160)
    out_lines.append("SWEEP DIAGNOSTIC — Hot Motor (Rs=0.110), True Step Response")
    out_lines.append("Run 1: Random Start Angle (Like real hardware)")
    out_lines.append("Run 2: Forced Start Angle (Aligned to 1.57 rad)")
    out_lines.append("=" * 160)
    out_lines.append("")

    for run_mode in ["RANDOM", "FORCED"]:
        force_align = (run_mode == "FORCED")
        out_lines.append(f"########## RUN MODE: {run_mode} ##########\n")
        total_rows = 0

        for thr in throttles:
            try:
                snaps, start_theta = run_one(lib, thr, force_align=force_align)
                out_lines.append(f"### TARGET THROTTLE = {thr:.2f}  |  START THETA = {start_theta:.2f} rad")
                out_lines.append(
                    f"{'Cycle':>7} | {'Tag':<12} | {'W_true':>7} | {'W_est':>7} | {'W_err':>7} | "
                    f"{'Ang_T':>5} | {'Ang_E':>5} | {'PhsErr':>6} | {'I_amp':>5} | {'Id':>6} | {'Iq':>6}"
                )
                out_lines.append("-" * 160)

                for s in snaps:
                    w_err = s["w_est"] - s["w_true"]
                    phs_err = wrap_pi(s["ang_t"] - s["ang_e"])
                    i_amp = math.sqrt(s["i_a"] ** 2 + s["i_b"] ** 2)
                    out_lines.append(
                        f"{s['cycle']:>7} | {s['tag']:<12} | {s['w_true']:>7.0f} | {s['w_est']:>7.0f} | "
                        f"{w_err:>7.0f} | {s['ang_t']:>5.2f} | {s['ang_e']:>5.2f} | "
                        f"{phs_err:>6.2f} | {i_amp:>5.1f} | {s['id']:>6.1f} | {s['iq']:>6.1f}"
                    )
                    total_rows += 1
            except Exception as e:
                out_lines.append(f"### TARGET THROTTLE = {thr:.2f}")
                out_lines.append(f"  CRASH: {e}")
            out_lines.append("")

    out_lines.append("=" * 160)
    out_lines.append("=" * 160)

    text = "\n".join(out_lines)
    out_file = Path(__file__).parent / "sweep_results.txt"
    out_file.write_text(text, encoding="utf-8")

    print(text)
    print(f"\n[+] Full results saved to: {out_file}")


if __name__ == "__main__":
    main()