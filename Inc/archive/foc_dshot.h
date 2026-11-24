#ifndef FOC_DSHOT_H
#define FOC_DSHOT_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    DSHOT_STATE_IDLE = 0,
    DSHOT_STATE_RX,
    DSHOT_STATE_GAP,
    DSHOT_STATE_ERROR,
} DShotPhase;

typedef struct {
    /* --- Public (read from FOC core) --- */
    volatile float    throttle;       /* 0.0 .. 1.0 */
    volatile bool     armed;
    volatile bool     bidir;
    volatile uint32_t frames_ok;
    volatile uint32_t frames_bad;

    /* Set by FOC core before next TX (v1: unused) */
    volatile uint32_t erpm_setpoint;

    /* --- Internal --- */
    DShotPhase _state;
    uint32_t   _rx_buf[48];
    volatile uint16_t _rx_count;
    uint32_t   _rx_started_us;
    uint32_t   _last_frame_us;
    uint32_t   _last_throttle_zero_us;
} DShotState;

extern DShotState dshot;

void dshot_init(void);
void dshot_poll(void);                       /* call every FOC cycle (24 kHz) */
void dshot_set_erpm(uint32_t erpm);          /* v1: stored only, v2: transmitted */

/* EXTI0_IRQHandler is defined in foc_dshot.c and declared by startup vector */

#endif /* FOC_DSHOT_H */