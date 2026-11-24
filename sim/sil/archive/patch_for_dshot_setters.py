"""Run from simulation/: python3 sim/sil/patch_for_dshot_setters.py"""
from pathlib import Path

H = Path(__file__).parent / "virtual_stm32.h"
C = Path(__file__).parent / "virtual_stm32.c"

# Add setters to header
h = H.read_text(encoding="utf-8")
if "sil_dshot_set_throttle" not in h:
    h = h.replace(
        "float sil_dshot_get_throttle(void); uint8_t sil_dshot_get_armed(void);",
        "float sil_dshot_get_throttle(void); uint8_t sil_dshot_get_armed(void);\n"
        "void sil_dshot_set_throttle(float t); void sil_dshot_set_armed(uint8_t a);"
    )
    H.write_text(h, encoding="utf-8")
    print(f"[fixed] {H.name}: added sil_dshot_set_* declarations")
else:
    print(f"[skip]  {H.name}: already has sil_dshot_set_*")

# Add setters to source
c = C.read_text(encoding="utf-8")
if "sil_dshot_set_throttle" not in c:
    c += (
        "\nvoid sil_dshot_set_throttle(float t){ dshot.throttle = t; }\n"
        "void sil_dshot_set_armed(uint8_t a){ dshot.armed = a ? true : false; }\n"
    )
    C.write_text(c, encoding="utf-8")
    print(f"[fixed] {C.name}: added sil_dshot_set_* implementations")
else:
    print(f"[skip]  {C.name}: already has sil_dshot_set_*")

print("[done]")