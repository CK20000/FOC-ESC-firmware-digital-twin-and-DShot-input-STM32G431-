# FOC ESC: firmware, digital twin and DShot input (STM32G431)

Sensorless field-oriented control (FOC) firmware for a custom 6S drone ESC, together with a software-in-the-loop (SIL) digital twin that runs the same C code against a simulated motor, inverter and PCB.

**Status:** the firmware and the DShot input have been tested on a bare STM32G431 dev board. The ESC PCB in this repository has passed DFM checks but has not been manufactured yet, so nothing here has been run on a motor.

## What is in this repository

- **Firmware (`Src/`, `Inc/`)**: 48 kHz sensorless FOC with a sliding-mode observer (SMO), a PLL, I-f startup and space vector PWM. The time-critical code runs from CCM RAM.
- **DShot300 receiver (`Src/foc_dshot.c`)**: a raw 32-edge decoder on PA0, written from scratch.
- **Digital twin (`sim/`)**: a C plant model (PMSM, inverter dead-time, Rds(on) drops, battery sag) plus a Python solver for PCB heat and ground bounce, shown in a PyQt6 dashboard.
- **Hardware (KiCad project and Gerbers)**: a 4-layer board (F.Cu, In1.Cu, In2.Cu, B.Cu) built around an STM32G431, an EG2134 gate driver and AD8418 current-sense amplifiers.

## Layout

```text
Inc/
  foc_core.h         FOC API, state machine, debug struct
  foc_dshot.h        DShot API and receiver state
Src/
  foc_core.c         FOC algorithm (SMO, PLL, I-f startup, CCM RAM)
  foc_dshot.c        DShot300 receiver
  main.c             Hardware init, CCM copy routine, flash-readout benchmark
  usb_cdc.c          USB CDC support
sim/
  sil/               Firmware + C plant, built into a shared library
    plant_c.c/.h              plant model
    sim_bridge.c/.h           bridge between the plant and Python
    virtual_stm32.c/.h        x86 stand-in for the MCU
    streaming_scope4.py       PyQt6 dashboard
  hil/               PCB thermal and ground-bounce solvers
    unified_esc_dashboard.py
    pcb_geometry.json
  test_dshot.py, virtual_mcu_dshot.py   DShot decoder tests on a virtual MCU
Makefile.hardware    Builds the .elf/.bin for the STM32G431
foc_g431.ld          Linker script with a CCM RAM section
```

## Quick start: simulation

```bash
cd sim/sil
make clean && make
python3 streaming_scope4.py
```

Press **Play**, raise the **Throttle** slider, and raise the **Speed** slider to speed up the simulation.

## Quick start: hardware

Install the ARM toolchain and a DFU tool:

```bash
sudo apt install gcc-arm-none-eabi dfu-util
```

Build from the folder that contains `Makefile.hardware`:

```bash
make -f Makefile.hardware clean
make -f Makefile.hardware -j4
```

This writes `build_hardware/foc_g431.elf`, `.bin` and `.map`.

Flash over USB DFU:

1. Hold **BOOT0**, tap **RESET**, release **BOOT0**.
2. Under WSL, attach the USB device first (Windows PowerShell, as admin): `usbipd list`, then `usbipd attach --wsl --busid <BUSID>`.
3. Flash:

```bash
sudo dfu-util --device 0483:df11 --alt 0 --dfuse-address 0x08000000:leave --download build_hardware/foc_g431.bin
```

The final `Error during download get_status` message is expected, since the board reboots after flashing.

## Measurements on silicon

`main.c` runs `foc_loop()` for 5 seconds (240,000 iterations), records the worst-case cycle count with the `DWT->CYCCNT` register, and saves a debug record to the last flash page (`0x0801F800`). Read it back with:

```bash
rm -f debug.bin
sudo dfu-util --device 0483:df11 --alt 0 --dfuse-address 0x0801F800:40 --upload debug.bin
hexdump -e '5/4 "%d " "\n"' debug.bin
```

The record is the `DebugInfo` struct in `foc_core.h`:

| Field | Meaning |
| :--- | :--- |
| `magic` | `0xDEADBEEF` if the record is valid |
| `loop_count` | Number of loop iterations run |
| `final_state` | Drive state at the end (0 = OFF, 3 = closed-loop FOC) |
| `throttle_x1000` | Throttle times 1000 (800 means 0.8) |
| `max_cycles` | Worst-case cycles for one loop |

Results so far, both on a dev board with no motor attached:

| Run | Final state | Throttle | Worst-case cycles | Time at 170 MHz |
| :--- | :--- | :--- | :--- | :--- |
| Closed-loop benchmark (forced closed-loop state) | 3 | 0.800 | 1,224 | 7.2 µs |
| DShot input test (DShot command from a flight controller) | 0 (OFF) | 0.480 | 1,162 | 6.8 µs |

The 1,224-cycle figure is the one to use for the closed-loop budget. The DShot run stayed in the OFF state, so its 1,162 cycles do not include the closed-loop control math. What the DShot run shows is that a throttle command sent by a flight controller reached the FOC core.

## DShot300 input

A DShot frame is 16 bits (11 throttle, 1 telemetry request, 4 CRC), which gives 32 edges per frame. At DShot300 each bit lasts 3.33 µs. The decoder on PA0 timestamps each edge with the DWT cycle counter.

Problems found and fixed while testing on the dev board:

- **Wrong frame length.** The first decoder waited for 42 edges and used GCR decoding. GCR is used only for the ESC's replies in bidirectional DShot, so the old decoder spanned two frames. It now decodes the raw 32-edge frame.
- **Clock speed.** The Arduino core ran the chip at 150 MHz, not 170 MHz, so a fixed 0/1 threshold failed. The decoder now measures the bit period and sets the threshold from it.
- **CRC.** With bidirectional DShot enabled, Betaflight inverts the CRC. The decoder accepts both forms.
- **Interrupt priority.** Every second frame collided with the 1 kHz SysTick interrupt, which delayed the edge timestamp. Giving the EXTI edge interrupt a higher priority than SysTick fixed it.

The receiver keeps `frames_ok` and `frames_bad` counters in the `dshot` struct. A complete frame-error measurement over a full throttle sweep is still to be published.

## Simulation features

- **I-f startup:** forces current into the motor and rotates the field to drag the rotor from rest.
- **SMO + PLL:** sensorless angle estimate.
- **Inverter model:** 200 ns dead-time distortion, MOSFET Rds(on) drops, battery sag.
- **Thermal model:** finite-difference heat diffusion across the PCB copper layers, from conduction (I²R) and switching losses based on FET datasheets.
- **Ground bounce:** copper resistance extracted from the layout, with differential ground-bounce errors injected into the ADC measurements.

## Pinout

| Pin | Function | Description |
| :--- | :--- | :--- |
| PA0 | DShot input | From the flight controller |
| PA1 | Phase U | ADC1 current sense |
| PA7 | Phase V | ADC2 current sense |
| PA8 | U high | Gate driver |
| PA9 | V high | Gate driver |
| PA10 | W high | Gate driver |
| PB13 | U low | Gate driver |
| PB14 | V low | Gate driver |
| PB15 | W low | Gate driver |
| PA2 / PA3 | Telemetry TX / RX | USART2 at 2 Mbaud |
| PA13 / PA14 | SWDIO / SWCLK | Debug |

Telemetry is a binary 49-byte struct on USART2. If the board is wired for USART1 on PB6/PB7 instead, switch `MX_USART2_UART_Init` to `MX_USART1_UART_Init` with `GPIO_PIN_6 | GPIO_PIN_7` and `AF7_USART1`.

## Board status and next steps

- The 4-layer PCB has passed DFM checks against JLCPCB capabilities.
- Components are being sourced for the first board.
- Next: assemble the first board, check the DShot timing on PA0 with a scope, and do a current-limited bench bring-up before any motor test.

## Known limits

- The gate driver, current sensing, observer and PLL have not been tested on a real motor.
- The digital twin is only as accurate as its motor and inverter model.
- The dev-board tests used no motor, so the timing figures come from benchmark inputs, not from a spinning rotor.
