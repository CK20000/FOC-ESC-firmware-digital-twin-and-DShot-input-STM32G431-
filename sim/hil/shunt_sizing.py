"""
shunt_thermal.py

Package-size thermal sizing for metal-strip shunts.
Brand-independent. Focuses on physical size and thermal path.

Two layout qualities are modelled per package:
  minimal  = shunt on its own pads, no extra copper
  good     = shunt on generous pads, solid copper pour attached

No brand-specific data. Just package dimensions and typical
conduction-aware theta_ja for each layout quality.
"""

import argparse

# ---------- Config ----------
R_SHUNT        = 0.002      # ohm
TCR_PPM        = 75.0       # ppm/C, typical metal strip
T_BODY_MAX     = 155.0      # C, coated metal strip limit
T_AMBIENT      = 80.0       # C, realistic ambient near power stage

# ---------- Package table ----------
# Each entry: (label, length_mm, width_mm, theta_ja_minimal, theta_ja_good)
# theta_ja_minimal  = pad size ≈ body footprint
# theta_ja_good     = pad extends ~5 mm each side, copper pour attached
# Values are typical for 2 mOhm metal-strip parts.
PACKAGES = [
    ("2512",   6.3,  3.2,  55.0,  35.0),
    ("2725",   6.9,  6.4,  48.0,  30.0),
    ("3920",  10.0,  5.0,  40.0,  22.0),
    ("5931",  15.0,  8.0,  28.0,  15.0),
]


def body_temp(i_rms, r_ohm, tcr_ppm, theta_ja, t_ambient):
    """Self-consistent iteration: P -> T -> R -> P..."""
    t_body = t_ambient + 10.0
    for _ in range(25):
        r_shifted = r_ohm * (1.0 + tcr_ppm * 1e-6 * (t_body - 25.0))
        p = i_rms * i_rms * r_shifted
        t_new = t_ambient + p * theta_ja
        if abs(t_new - t_body) < 0.05:
            t_body = t_new
            break
        t_body = t_new
    r_shifted = r_ohm * (1.0 + tcr_ppm * 1e-6 * (t_body - 25.0))
    p = i_rms * i_rms * r_shifted
    drift_pct = (r_shifted / r_ohm - 1.0) * 100.0
    return {
        "t_body":    t_body,
        "t_rise":    t_body - t_ambient,
        "p_diss":    p,
        "drift_pct": drift_pct,
        "pass":      t_body <= T_BODY_MAX,
    }


def max_rms_current(r_ohm, tcr_ppm, theta_ja, t_ambient):
    lo, hi = 1.0, 300.0
    for _ in range(45):
        mid = 0.5 * (lo + hi)
        if body_temp(mid, r_ohm, tcr_ppm, theta_ja, t_ambient)["pass"]:
            lo = mid
        else:
            hi = mid
    return lo


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--current", type=float, default=40.0,
                    help="target phase current (default 40 A)")
    ap.add_argument("--peak", action="store_true",
                    help="current is peak, not RMS (divides by sqrt(2))")
    ap.add_argument("--ambient", type=float, default=T_AMBIENT)
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    T_AMB = args.ambient
    target_rms = args.current / (2.0 ** 0.5) if args.peak else args.current

    print("=" * 78)
    print("  SHUNT THERMAL — package size only")
    print("=" * 78)
    print()
    print(f"  R_shunt:      {R_SHUNT*1e3:.2f} mΩ")
    print(f"  TCR:          {TCR_PPM:.0f} ppm/°C")
    print(f"  T_body_max:   {T_BODY_MAX:.0f} C")
    print(f"  T_ambient:    {T_AMB:.0f} C")
    print(f"  Target:       {args.current:.0f} A "
          f"({'peak' if args.peak else 'rms'} = "
          f"{target_rms:.1f} A_rms)")
    print()

    # --- Body temp table (good layout) ---
    currents = [10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60]
    print("=" * 78)
    print("  Body temperature (C) — GOOD layout (pads + copper pour)")
    print("=" * 78)
    print()
    hdr = f"  {'I_rms':>6s} |"
    for label, _, _, _, _ in PACKAGES:
        hdr += f" {label:>10s} |"
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))
    for i in currents:
        row = f"  {i:>4}A |"
        for label, _, _, _, tj_good in PACKAGES:
            r = body_temp(float(i), R_SHUNT, TCR_PPM, tj_good, T_AMB)
            mark = " " if r["pass"] else "!"
            row += f" {r['t_body']:>8.0f}{mark} |"
        print(row)
    print("  (! = exceeds T_body_max)")
    print()

    # --- Body temp table (minimal layout) ---
    print("=" * 78)
    print("  Body temperature (C) — MINIMAL layout (pad only)")
    print("=" * 78)
    print()
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))
    for i in currents:
        row = f"  {i:>4}A |"
        for label, _, _, tj_min, _ in PACKAGES:
            r = body_temp(float(i), R_SHUNT, TCR_PPM, tj_min, T_AMB)
            mark = " " if r["pass"] else "!"
            row += f" {r['t_body']:>8.0f}{mark} |"
        print(row)
    print("  (! = exceeds T_body_max)")
    print()

    # --- Max continuous current per package ---
    print("=" * 78)
    print("  Max continuous RMS phase current")
    print("=" * 78)
    print()
    print(f"  {'Package':>8s}  {'Body (mm)':>12s}  "
          f"{'θja_good':>10s}  {'I_max_good':>12s}  "
          f"{'θja_min':>9s}  {'I_max_min':>11s}")
    print(f"  {'-'*8}  {'-'*12}  {'-'*10}  {'-'*12}  "
          f"{'-'*9}  {'-'*11}")
    for label, x, y, tj_min, tj_good in PACKAGES:
        i_max_good = max_rms_current(R_SHUNT, TCR_PPM, tj_good, T_AMB)
        i_max_min  = max_rms_current(R_SHUNT, TCR_PPM, tj_min,  T_AMB)
        print(f"  {label:>8s}  {x:>5.1f}x{y:<5.1f}  "
              f"{tj_good:>8.0f}    {i_max_good:>10.1f}    "
              f"{tj_min:>7.0f}    {i_max_min:>9.1f}")
    print()

    # --- Focus on target current ---
    print("=" * 78)
    print(f"  At target {args.current:.0f} A "
          f"({'peak' if args.peak else 'rms'} = {target_rms:.1f} A_rms)")
    print("=" * 78)
    print()
    print(f"  {'Package':>8s}  {'P_diss':>8s}  {'dT_good':>8s}  "
          f"{'T_body_good':>12s}  {'dT_min':>7s}  {'T_body_min':>11s}  Status")
    print(f"  {'-'*8}  {'-'*8}  {'-'*8}  {'-'*12}  "
          f"{'-'*7}  {'-'*11}  {'-'*6}")
    for label, _, _, tj_min, tj_good in PACKAGES:
        r_good = body_temp(target_rms, R_SHUNT, TCR_PPM, tj_good, T_AMB)
        r_min  = body_temp(target_rms, R_SHUNT, TCR_PPM, tj_min,  T_AMB)
        if r_good["pass"]:
            status = "OK"
        elif r_min["pass"]:
            status = "needs good pads"
        else:
            status = "FAIL"
        print(f"  {label:>8s}  {r_good['p_diss']:>6.2f}W  "
              f"{r_good['t_rise']:>6.0f}C  {r_good['t_body']:>10.0f}C  "
              f"{r_min['t_rise']:>5.0f}C  {r_min['t_body']:>9.0f}C  "
              f"{status}")
    print()

    # --- Layout guidance per package ---
    print("=" * 78)
    print("  Layout guidance per package (for good thermal)")
    print("=" * 78)
    print()
    print(f"  {'Package':>8s}  {'Pad size (mm)':>14s}  "
          f"{'Pad area (mm²)':>14s}  {'Vias per pad':>12s}")
    print(f"  {'-'*8}  {'-'*14}  {'-'*14}  {'-'*12}")
    for label, x, y, _, _ in PACKAGES:
        # Good pads: extend body length by 3 mm each side
        pad_l = x + 6.0
        pad_w = y + 1.0
        pad_area = pad_l * pad_w
        if label == "2512":
            vias = 4
        elif label in ("2725", "3920"):
            vias = 6
        else:
            vias = 9
        print(f"  {label:>8s}  {pad_l:>4.1f}x{pad_w:<8.1f}  "
              f"{pad_area:>12.0f}    {vias:>10d}")
    print()

    if args.plot:
        try:
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(10, 6))
            xs = [x * 0.5 for x in range(4, 141)]
            for label, _, _, tj_min, tj_good in PACKAGES:
                ys_good = [body_temp(x, R_SHUNT, TCR_PPM, tj_good, T_AMB)["t_body"]
                           for x in xs]
                ys_min  = [body_temp(x, R_SHUNT, TCR_PPM, tj_min,  T_AMB)["t_body"]
                           for x in xs]
                line, = ax.plot(xs, ys_good, label=f"{label} good layout")
                ax.plot(xs, ys_min, color=line.get_color(),
                        ls="--", alpha=0.5,
                        label=f"{label} minimal")
            ax.axhline(T_BODY_MAX, color="r", ls="--",
                       label=f"T_body_max = {T_BODY_MAX:.0f}C")
            ax.axhline(T_AMB, color="gray", ls=":", label=f"Ambient")
            ax.axvline(target_rms, color="k", ls=":", alpha=0.5,
                       label=f"Target {target_rms:.0f}A_rms")
            ax.set_xlabel("Phase RMS current (A)")
            ax.set_ylabel("Shunt body temperature (C)")
            ax.set_title(f"Shunt body temp vs current — {R_SHUNT*1e3:.1f} mΩ")
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=8, ncol=2)
            plt.tight_layout()
            plt.savefig("shunt_thermal.png", dpi=120)
            print("  [+] Wrote shunt_thermal.png")
        except ImportError:
            print("  [!] matplotlib not installed — skipping plot")


if __name__ == "__main__":
    main()