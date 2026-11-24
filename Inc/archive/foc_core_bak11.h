#ifndef FOC_CORE_H
#define FOC_CORE_H

#include <stdint.h>
#include <stdbool.h>

/* Motor DNA */
#define FOC_RS         0.090f
#define FOC_LS         0.00003f
#define FOC_LAMBDA     0.00041f
#define FOC_POWER_SUPPLY   25.2f
#define FOC_LOOP_DT    20.833333e-6f /* 48 kHz */
#define FOC_AMPS_PER_COUNT  ((3.3f / 4095.0f) / (0.002f * 20.0f))

/* --- PRE-CALCULATED RECIPROCALS (Saves 14-cycle FPU Divides) --- */
#define INV_LS         (1.0f / FOC_LS)
#define INV_SUPPLY     (1.0f / FOC_POWER_SUPPLY)
#define INV_SQRT3      0.577350269f
#define INV_3_0        (1.0f / 3.0f)
#define INV_4000       (1.0f / 4000.0f)
#define TIM1_ARR       1770.0f /* 170MHz / (2 * 48kHz) */

/* State machine */
typedef enum {
    STATE_OFF = 0,
    STATE_ALIGN,
    STATE_FOC_BLEND,
    STATE_CLOSED_LOOP_FOC,
    STATE_KICK_START,
} DriveState;

void foc_setup(void);
void foc_loop(void);
void foc_set_adc_offsets(float u, float v);

DriveState foc_get_state(void);
float foc_get_estimated_speed(void);
float foc_get_estimated_angle(void);
float foc_get_phase_u_current(void);
float foc_get_phase_v_current(void);
float foc_get_v_alpha(void);
float foc_get_v_beta(void);
float foc_get_pll_integ(void);

/* --- DEBUG STRUCT FOR FLASH READBACK --- */
typedef struct {
    uint32_t magic;          // 0xDEADBEEF if valid
    uint32_t loop_count;     // How many loops ran
    uint32_t final_state;    // The state at the end (4 = CLOSED_LOOP_FOC)
    uint32_t throttle_x1000; // Throttle * 1000 (800 means 0.8)
    uint32_t max_cycles;     // Max compute cycles
} DebugInfo;

extern volatile DebugInfo foc_debug;

#endif