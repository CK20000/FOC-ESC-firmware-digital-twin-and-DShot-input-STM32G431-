"""
fet_thermal_envelope.py

Pre-layout thermal sizing for the FOC ESC power stage.

No PCB geometry needed. Answers:
    "Given phase RMS current X, what copper area does each FET need
     to stay below Tj_target?"

Model:
- AOD4184A datasheet-fit Rds_on and Qgd
- EG2134 gate drive, 10Ω per-FET gate resistor
- Conduction loss via standard 3-phase PWM formulas
- Switching loss via gate-charge-driven crossover time
- Self-consistent iteration (Rds_on depends on Tj, Tj depends on Rds_on)
- IPC-2152 style theta_ja vs copper area table for TO-252

Outputs:
- Tj table: phase current x copper area
- Max safe phase current per copper area
- Loss breakdown at a reference current
- Optional plot (--plot)
"""

import argparse
import math

# ---------- FET parameters (AOD4184A datasheet-fit) ----------
FET = {
    "name":         "AOD4184A",
    "package":      "TO-252",
    "vds_max":      40.0,
    "rds_on_25c":   6.7e-3,     # ohm, Vgs=10V
    "rds_on_100c":  9.8e-3,     # ohm, Vgs=10V, ~1.45x
    "rds_on_125c":  10.9e-3,
    "qgd":          12e-9,      # C, gate-drain (Miller) charge
}

# ---------- Gate drive ----------
GATE_DRIVE = {
    "ic":           "EG2134",
    "i_source":     1.2,        # A
    "i_sink":       1.4,        # A
    "v_drive":      12.0,       # V
    "r_gate":       10.0,       # ohm, per-FET series gate resistor
    "r_gate_ic":    2.0,        # ohm, IC output impedance
}

# ---------- System ----------
SYSTEM = {
    "v_bus":            16.8,   # V (4S full)
    "f_sw":             24e3,   # Hz
    "modulation":       0.8,    # SVM index (0..1.15)
    "power_factor":     1.0,    # FOC: current aligned with back-EMF
    "fets_parallel":    2,
    "ambient":          25.0,   # C
    "tj_target":        125.0,  # C, design margin (83% of 150C abs)
}

# theta_ja vs copper area (IPC-2152 style, TO-252, 1oz FR4)
COPPER_TABLE = [
    (25.0,   60.0),
    (50.0,   50.0),
    (100.0,  40.0),
    (250.0,  32.0),
    (500.0,  25.0),
]


def rds_on_at_tj(tj):
    r25, r100, r125 = (FET["rds_on_25c"], FET["rds_on_100c"],
                       FET["rds_on_125c"])
    if tj <= 25:
        return r25
    if tj <= 100:
        return r25 + (r100 - r25) * (tj - 25) / 75.0
    if tj <= 125:
        return r100 + (r125 - r100) * (tj - 100) / 25.0
    slope = (r125 - r100) / 25.0
    return r125 + slope * (tj - 125.0)


def gate_rise_fall_times():
    gd = GATE_DRIVE
    i_plateau = gd["v_drive"] / (gd["r_gate"] + gd["r_gate_ic"])
    i_on  = min(i_plateau, gd["i_source"])
    i_off = min(i_plateau, gd["i_sink"])
    t_rise = FET["qgd"] / i_on
    t_fall = FET["qgd"] / i_off
    return t_rise, t_fall


def conduction_loss_per_fet(i_peak_phase, rds_on, m, cosphi, n_parallel):
    """Standard 3-phase PWM conduction loss, per FET."""
    coeff_hs = 1.0/8.0 + m * cosphi / (3.0 * math.pi)
    coeff_ls = 1.0/8.0 - m * cosphi / (3.0 * math.pi)
    n2 = n_parallel * n_parallel
    p_hs = i_peak_phase**2 * rds_on * coeff_hs / n2
    p_ls = i_peak_phase**2 * rds_on * coeff_ls / n2
    return p_hs, p_ls


def switching_loss_per_fet(i_rms_phase, n_parallel, v_bus, f_sw,
                           t_rise, t_fall):
    """Switching loss averaged over sine cycle, per FET."""
    i_peak_phase = i_rms_phase * math.sqrt(2.0)
    i_fet_peak = i_peak_phase / n_parallel
    p_peak = v_bus * i_fet_peak * f_sw * (t_rise + t_fall) / 2.0
    return (2.0 / math.pi) * p_peak


def solve(i_rms_phase, theta_ja, tj_target):
    """Self-consistent iteration. Returns dict."""
    ambient = SYSTEM["ambient"]
    tj = ambient + 20.0
    rds = rds_on_at_tj(tj)
    p_hs = p_ls = p_sw = p_tot = 0.0

    for _ in range(30):
        rds = rds_on_at_tj(tj)
        t_rise, t_fall = gate_rise_fall_times()
        p_hs, p_ls = conduction_loss_per_fet(
            i_rms_phase * math.sqrt(2.0), rds,
            SYSTEM["modulation"], SYSTEM["power_factor"],
            SYSTEM["fets_parallel"])
        p_sw = switching_loss_per_fet(
            i_rms_phase, SYSTEM["fets_parallel"],
            SYSTEM["v_bus"], SYSTEM["f_sw"], t_rise, t_fall)
        p_tot = p_hs + p_sw
        tj_new = ambient + p_tot * theta_ja
        if abs(tj_new - tj) < 0.05:
            tj = tj_new
            break
        tj = tj_new

    return {
        "i_rms":       i_rms_phase,
        "theta_ja":    theta_ja,
        "rds_mohm":    rds * 1e3,
        "p_cond_hs":   p_hs,
        "p_cond_ls":   p_ls,
        "p_sw":        p_sw,
        "p_total":     p_tot,
        "tj":          tj,
        "margin":      tj_target - tj,
        "pass":        tj <= tj_target,
    }


def max_safe_current(theta_ja, tj_target):
    lo, hi = 1.0, 200.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        r = solve(mid, theta_ja, tj_target)
        if r["pass"]:
            lo = mid
        else:
            hi = mid
    return lo


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ambient", type=float, default=SYSTEM["ambient"])
    ap.add_argument("--tj-max",  type=float, default=SYSTEM["tj_target"])
    ap.add_argument("--v-bus",   type=float, default=SYSTEM["v_bus"])
    ap.add_argument("--f-sw",    type=float, default=SYSTEM["f_sw"])
    ap.add_argument("--plot",    action="store_true")
    args = ap.parse_args()

    SYSTEM["ambient"]   = args.ambient
    SYSTEM["tj_target"] = args.tj_max
    SYSTEM["v_bus"]     = args.v_bus
    SYSTEM["f_sw"]      = args.f_sw

    t_rise, t_fall = gate_rise_fall_times()

    print("=" * 74)
    print("  FET THERMAL ENVELOPE — pre-layout sizing")
    print("=" * 74)
    print()
    print(f"  FET:          {FET['name']} ({FET['package']})")
    print(f"  Paralleled:   {SYSTEM['fets_parallel']}x per switch")
    print(f"  V_bus:        {SYSTEM['v_bus']:.2f} V")
    print(f"  f_sw:         {SYSTEM['f_sw']/1e3:.1f} kHz")
    print(f"  Mod index:    {SYSTEM['modulation']:.2f}  "
          f"cosphi={SYSTEM['power_factor']:.2f}")
    print(f"  Rds_on(25C):  {FET['rds_on_25c']*1e3:.2f} mΩ")
    print(f"  Rds_on(100C): {FET['rds_on_100c']*1e3:.2f} mΩ")
    print(f"  Qgd:          {FET['qgd']*1e9:.1f} nC")
    print(f"  Gate:         {GATE_DRIVE['ic']}, "
          f"R_gate={GATE_DRIVE['r_gate']:.0f}Ω")
    print(f"  t_rise:       {t_rise*1e9:.1f} ns")
    print(f"  t_fall:       {t_fall*1e9:.1f} ns")
    print(f"  Ambient:      {SYSTEM['ambient']:.1f} C")
    print(f"  Tj_target:    {SYSTEM['tj_target']:.1f} C")
    print()

    # --- Tj table ---
    currents = list(range(5, 65, 5))
    print("=" * 74)
    print("  Tj (C) — HS FET, worst case, vs phase RMS current")
    print("=" * 74)
    print()
    hdr = f"  {'I_rms':>6s} |"
    for area, _ in COPPER_TABLE:
        hdr += f" {area:>5.0f}mm² |"
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))

    for i in currents:
        row = f"  {i:>4}A |"
        for area, theta in COPPER_TABLE:
            r = solve(float(i), theta, SYSTEM["tj_target"])
            cell = f" {r['tj']:>5.0f}  "
            if not r["pass"]:
                cell = f" {r['tj']:>4.0f}!"  # mark failure
                cell = f"{cell:>7s}"
            row += cell + "|"
        print(row)
    print("  (! = exceeds Tj_target)")

    # --- Max current per copper area ---
    print()
    print("=" * 74)
    print("  Max safe phase current (Tj < Tj_target)")
    print("=" * 74)
    print()
    print(f"  {'Copper area':>12s}  {'θja':>8s}  {'I_max (A_rms)':>14s}")
    print(f"  {'-'*12}  {'-'*8}  {'-'*14}")
    for area, theta in COPPER_TABLE:
        i_max = max_safe_current(theta, SYSTEM["tj_target"])
        print(f"  {area:>10.0f}mm²  {theta:>6.0f}C/W  {i_max:>14.1f}")

    # --- Loss breakdown at 30 A ---
    print()
    print("=" * 74)
    print("  Loss breakdown at 30 A_rms")
    print("=" * 74)
    print()
    print(f"  {'Area':>7s}  {'Rds_on':>9s}  {'P_cond':>9s}  "
          f"{'P_sw':>9s}  {'P_total':>9s}  {'Tj':>8s}")
    print(f"  {'-'*7}  {'-'*9}  {'-'*9}  {'-'*9}  {'-'*9}  {'-'*8}")
    for area, theta in COPPER_TABLE:
        r = solve(30.0, theta, SYSTEM["tj_target"])
        print(f"  {area:>5.0f}mm²  {r['rds_mohm']:>7.2f}mΩ  "
              f"{r['p_cond_hs']:>7.3f}W  {r['p_sw']:>7.3f}W  "
              f"{r['p_total']:>7.3f}W  {r['tj']:>6.1f}C")

    if args.plot:
        try:
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(10, 6))
            for area, theta in COPPER_TABLE:
                tjs = []
                xs = []
                for i in [x * 0.5 for x in range(4, 121)]:
                    r = solve(i, theta, SYSTEM["tj_target"])
                    xs.append(i)
                    tjs.append(r["tj"])
                ax.plot(xs, tjs, label=f"{area:.0f}mm², θja={theta:.0f}")
            ax.axhline(SYSTEM["tj_target"], color="r", ls="--",
                       label=f"Tj_target = {SYSTEM['tj_target']:.0f}C")
            ax.axhline(150, color="darkred", ls=":", label="Tj abs max")
            ax.set_xlabel("Phase RMS current (A)")
            ax.set_ylabel("HS FET junction temperature (C)")
            ax.set_title("FET thermal envelope")
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=9)
            plt.tight_layout()
            plt.savefig("fet_thermal_envelope.png", dpi=120)
            print()
            print("  [+] Wrote fet_thermal_envelope.png")
        except ImportError:
            print("  [!] matplotlib not installed — skipping plot")


if __name__ == "__main__":
    main()