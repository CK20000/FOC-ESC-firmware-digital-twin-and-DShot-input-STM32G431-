import ctypes
from pathlib import Path

so = Path(__file__).parent / "foc_sim.so"
lib = ctypes.CDLL(str(so))

lib.foc_set_adc_offsets.argtypes = [ctypes.c_float, ctypes.c_float]
lib.sil_adc1_set_jdr1.argtypes = [ctypes.c_uint32]
lib.sil_adc2_set_jdr1.argtypes = [ctypes.c_uint32]
lib.sil_tim1_get_ccr1.restype = ctypes.c_uint32
lib.sil_tim1_get_ccr2.restype = ctypes.c_uint32
lib.sil_tim1_get_ccr3.restype = ctypes.c_uint32
lib.sil_tim2_advance.argtypes = [ctypes.c_uint32]
lib.sil_dshot_get_throttle.restype = ctypes.c_float

print("[*] foc_set_adc_offsets + foc_setup")
lib.foc_set_adc_offsets(2048.0, 2048.0)
lib.foc_setup()
print("[+] setup OK")

print("[*] 100 foc_loop() calls, throttle=0")
ccrs = []
for i in range(100):
    lib.sil_adc1_set_jdr1(2048)
    lib.sil_adc2_set_jdr1(2048)
    lib.sil_tim2_advance(567)
    lib.foc_loop()
    ccrs.append((lib.sil_tim1_get_ccr1(), lib.sil_tim1_get_ccr2(), lib.sil_tim1_get_ccr3()))

print(f"[+] 100 loops ran")
print(f"    CCR1 range: {min(c[0] for c in ccrs)} .. {max(c[0] for c in ccrs)}")
print(f"    CCR2 range: {min(c[1] for c in ccrs)} .. {max(c[1] for c in ccrs)}")
print(f"    CCR3 range: {min(c[2] for c in ccrs)} .. {max(c[2] for c in ccrs)}")
print(f"    throttle  : {lib.sil_dshot_get_throttle():.3f}")
print("[+] SIL proof complete")
