#include "foc_dshot.h"
#ifdef HOST_BUILD
#include "virtual_stm32.h"
#else
#include "stm32g4xx.h"
#endif

DShotState dshot = {0};

/* ============================================================
 * DShot300 timing — 170 MHz kernel clock
 * ============================================================ */
#define F_CPU_HZ             170000000UL
#define TICKS_PER_US         (F_CPU_HZ / 1000000UL)

#define RX_BIT_PERIOD_TICKS  567     /* 3333 ns */
#define RX_THRESHOLD_TICKS   319     /* 1875 ns = true midpoint */

#define FRAME_TIMEOUT_US     1000    /* 1 ms → bad frame */
#define FRAMESAFE_TIMEOUT_US 100000  /* 100 ms → failsafe */

/* ============================================================
 * GCR encoding tables (DShot spec)
 * ============================================================ */
static const uint8_t gcr_encode[16] __attribute__((unused)) = {
    0x19,0x1B,0x12,0x13,0x1D,0x15,0x16,0x17,
    0x1A,0x09,0x0B,0x1E,0x0D,0x0E,0x0F,0x07
};

static const uint8_t gcr_decode[32] = {
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x0F,
    0xFF,0x09,0xFF,0x0A,0xFF,0x0C,0x0D,0x0E,
    0xFF,0xFF,0x02,0x03,0xFF,0x05,0x06,0x07,
    0xFF,0x00,0x08,0x01,0xFF,0x04,0x0B,0xFF
};

/* Forward declarations of internal helpers */
static bool _decode_frame(const uint32_t *ts, uint16_t n, uint16_t *out);

/* ============================================================
 * Init — PA0 EXTI on both edges + TIM2 free-running timestamp
 * ============================================================ */
void dshot_init(void) {
    /* 1. PA0 = input, pull-up */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    GPIOA->MODER  = (GPIOA->MODER  & ~GPIO_MODER_MODE0)
                  |  (0UL << GPIO_MODER_MODE0_Pos);
    GPIOA->PUPDR  = (GPIOA->PUPDR  & ~GPIO_PUPDR_PUPD0)
                  |  (1UL << GPIO_PUPDR_PUPD0_Pos);

    /* 2. SYSCFG: EXTI line 0 → PA0 */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~SYSCFG_EXTICR1_EXTI0)
                      |  (0UL << SYSCFG_EXTICR1_EXTI0_Pos);

    /* 3. EXTI0: both edges, interrupt enabled, clear pending */
    EXTI->IMR1  |= EXTI_IMR1_IM0;
    EXTI->RTSR1 |= EXTI_RTSR1_RT0;
    EXTI->FTSR1 |= EXTI_FTSR1_FT0;
    EXTI->PR1    = EXTI_PR1_PIF0;

    /* 4. TIM2: free-running 32-bit @ 170 MHz (timestamp source) */
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    TIM2->CR1   = 0;
    TIM2->PSC   = 0;
    TIM2->ARR   = 0xFFFFFFFFUL;
    TIM2->EGR   = TIM_EGR_UG;
    TIM2->CR1   = TIM_CR1_CEN;

    /* 5. NVIC — DShot must preempt FOC loop */
    NVIC_SetPriority(EXTI0_IRQn, 2);
    NVIC_EnableIRQ(EXTI0_IRQn);

    /* 6. State */
    dshot._state          = DSHOT_STATE_RX;
    dshot._rx_count       = 0;
    dshot._rx_started_us  = TIM2->CNT / TICKS_PER_US;
    dshot._last_frame_us  = 0;
    dshot._last_throttle_zero_us = 0;
    dshot.throttle        = 0.0f;
    dshot.armed           = false;
    dshot.bidir           = false;
    dshot.frames_ok       = 0;
    dshot.frames_bad      = 0;
    dshot.erpm_setpoint   = 0;
}

/* ============================================================
 * Called every FOC cycle (24 kHz). Decodes when frame is complete.
 * ============================================================ */
void dshot_poll(void) {
    uint32_t now_us = TIM2->CNT / TICKS_PER_US;

    /* Frame complete → decode */
    if (dshot._rx_count >= 32) {
        NVIC_DisableIRQ(EXTI0_IRQn);
        uint16_t frame;
        if (_decode_frame(dshot._rx_buf, 32, &frame)) {
            dshot.throttle = (frame >> 1) / 2047.0f;
            if (frame & 1) dshot.bidir = true;
            dshot.frames_ok++;
            dshot._last_frame_us = now_us;
        } else {
            dshot.frames_bad++;
        }
        dshot._rx_count = 0;
        dshot._rx_started_us = now_us;
        NVIC_EnableIRQ(EXTI0_IRQn);
    }

    /* Failsafe: no valid frame for 100 ms */
    if (dshot._last_frame_us != 0 &&
        (now_us - dshot._last_frame_us) > FRAMESAFE_TIMEOUT_US) {
        dshot.throttle = 0.0f;
    }

    /* Arm latch: 100 ms at zero throttle */
    if (dshot.throttle < 0.02f) {
        if (dshot._last_throttle_zero_us == 0) {
            dshot._last_throttle_zero_us = now_us;
        } else if ((now_us - dshot._last_throttle_zero_us) > 100000UL) {
            dshot.armed = true;
        }
    } else {
        dshot._last_throttle_zero_us = 0;
    }

    /* Partial frame stuck > 1 ms → discard, restart */
    if (dshot._rx_count > 0 && dshot._rx_count < 32 &&
        (now_us - dshot._rx_started_us) > FRAME_TIMEOUT_US) {
        dshot.frames_bad++;
        dshot._rx_count = 0;
        dshot._rx_started_us = now_us;
    }
}

void dshot_set_erpm(uint32_t erpm) {
    dshot.erpm_setpoint = erpm;   /* v2 will transmit this */
}

/* ============================================================
 * EXTI0 ISR — fires on every edge of PA0
 * ============================================================ */
void EXTI0_IRQHandler(void) {
    if (EXTI->PR1 & EXTI_PR1_PIF0) {
        EXTI->PR1 = EXTI_PR1_PIF0;                     /* clear pending */
        if (dshot._rx_count < 32) {
            dshot._rx_buf[dshot._rx_count++] = TIM2->CNT;
        }
    }
}

/* ============================================================
 * Decode 32 edge timestamps → 16-bit DShot frame
 * ============================================================ */
static bool _decode_frame(const uint32_t *ts, uint16_t n, uint16_t *out) {
    if (n < 32) return false;

    /* Reconstruct 16 bits from 16 high periods (rising→falling) */
    uint16_t frame = 0;
    for (int i = 0; i < 16; i++) {
        uint32_t high = ts[2*i + 1] - ts[2*i];
        frame = (frame << 1) | (high > RX_THRESHOLD_TICKS ? 1u : 0u);
    }

    /* DShot command: [15:5]=throttle, [4]=telem request, [3:0]=CRC */
    uint16_t tt       = frame >> 4;   /* 12 bits: [11:1] throttle, [0] telem */
    uint16_t crc_rx   = frame & 0x0F;
    uint16_t crc_calc = (tt ^ (tt >> 4) ^ (tt >> 8)) & 0x0F;
    if (crc_calc != crc_rx) return false;

    *out = tt;   /* caller's (frame >> 1) / 2047 and (frame & 1) still work */
    return true;
}