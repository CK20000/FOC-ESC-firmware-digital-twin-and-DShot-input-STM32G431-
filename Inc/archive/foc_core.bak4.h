#ifndef FOC_CORE_H
#define FOC_CORE_H

#include <stdint.h>
#include <stdbool.h>

/* Motor DNA — VERIFY on your specific motor */
#define FOC_RS         0.090f
#define FOC_LS         0.00003f
#define FOC_LAMBDA     0.00041f
#define FOC_POWER_SUPPLY   25.2f
#define FOC_LOOP_DT    50e-6f
#define FOC_AMPS_PER_COUNT  ((3.3f / 4095.0f) / (0.002f * 20.0f))

/* State machine */
typedef enum {
    STATE_OFF = 0,
    STATE_ALIGN,
    STATE_OPEN_LOOP_RAMP,
    STATE_FOC_BLEND,
    STATE_CLOSED_LOOP_FOC,
} DriveState;

/* Called once from main() after HAL_Init() + MX_*_Init() */
void foc_setup(void);

/* Called from main() while(1) loop — does one FOC cycle */
void foc_loop(void);

void foc_set_adc_offsets(float u, float v);

/* Debug accessors (used by telemetry) */
DriveState foc_get_state(void);
float foc_get_estimated_speed(void);
float foc_get_estimated_angle(void);
float foc_get_phase_u_current(void);
float foc_get_phase_v_current(void);
float foc_get_v_alpha(void);
float foc_get_v_beta(void);

#endif
