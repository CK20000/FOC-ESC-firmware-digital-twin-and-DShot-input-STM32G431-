import ctypes
import math
from pathlib import Path
from plant import PMSPlant
from run_foc import load_firmware

STATE_NAMES = ["OFF", "ALIGN", "BLEND", "CLOSED", "KICK"]


def main():
    so_path = Path(__file__).parent / "foc_sim.so"
    if not so_path.exists():
        print(f"[!] {so_path} not found — run `make` first")
        return

    lib = load_firmware(so_path)

    # Note: PMSPlant now uses RS=0.110 by default
    plant = PMSPlant(
        RS=0.110, LS=30e-6, LAMBDA=0.00041, POLE_PAIRS=7,
        INERTIA=5e-6, V_BUS=25.2, BETA_SIGN=+1, DAMPING=2e-5,
        PROP_LOAD=1e-8, seed=12345
    )

    print("=" * 120)
    print("IDLE DIAGNOSTIC — Calibration, Sensor Offsets, Noise, and 2-Cycle Delay")
    print("=" * 120)

    lib.foc_set_adc_offsets(2048.0, 2048.0)
    lib.foc_setup()
    lib.sil_dshot_set_armed(1)
    lib.sil_dshot_set_throttle(0.0)

    # --- ADDED: True 4000-cycle Calibration Loop ---
    print("Calibrating ADC offsets (4000 samples)...")
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

    mean_u = sum_u / 4000.0
    mean_v = sum_v / 4000.0
    print(f"Calibrated Hardware Offsets: U={mean_u:.1f}, V={mean_v:.1f}")
    lib.foc_set_adc_offsets(mean_u, mean_v)

    print("-" * 120)
    print(
        f"{'Cycle':>6} | {'State':<8} | {'ADC_U':>5} | {'ADC_V':>5} | {'I_U_FW':>7} | {'I_V_FW':>7} | {'PLL_Int':>8} | {'W_true':>6} | {'CCR1':>5}")
    print("-" * 120)

    for cycle in range(2000):
        ccr1 = lib.sil_tim1_get_ccr1()
        ccr2 = lib.sil_tim1_get_ccr2()
        ccr3 = lib.sil_tim1_get_ccr3()

        plant.step(ccr1, ccr2, ccr3)
        cnt_u, cnt_v = plant.adc_counts()
        lib.sil_adc1_set_jdr1(cnt_u)
        lib.sil_adc2_set_jdr1(cnt_v)
        lib.sil_tim2_advance(3541)
        lib.foc_loop()

        if cycle % 200 == 0:
            state = lib.foc_get_state()
            state_name = STATE_NAMES[state] if 0 <= state < len(STATE_NAMES) else "?"

            i_u_fw = lib.foc_get_phase_u_current()
            i_v_fw = lib.foc_get_phase_v_current()
            pll_int = lib.foc_get_pll_integ()
            w_true = plant.omega

            print(
                f"{cycle:>6} | {state_name:<8} | {cnt_u:>5} | {cnt_v:>5} | {i_u_fw:>7.2f} | {i_v_fw:>7.2f} | {pll_int:>8.2f} | {w_true:>6.1f} | {ccr1:>5}")

    print("=" * 120)


if __name__ == "__main__":
    main()