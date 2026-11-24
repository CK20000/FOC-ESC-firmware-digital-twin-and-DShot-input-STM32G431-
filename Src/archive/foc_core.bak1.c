/*
 * foc_core.c — FOC engine
 *
 * Observer: Sliding-Mode Observer (SMO) + PLL angle tracker.
 * Replaces the previous voltage-mode flux integrator.
 */
#include "foc_core.h"
#include "foc_dshot.h"
#ifdef HOST_BUILD
#include "virtual_stm32.h"
#else
#include "stm32g4xx.h"
#endif
#include <math.h>

#define _PI  3.14159265359f
#define _2PI 6.28318530718f

/* --- Control state --- */
static DriveState current_state = STATE_OFF;

static float cmd_v_alpha = 0.0f, cmd_v_beta = 0.0f, cmd_vq_throttle = 0.0f;
static float estimated_angle = 0.0f, estimated_speed = 0.0f;
static float open_loop_speed = 0.0f, open_loop_angle = 0.0f;

static float last_I_U = 0.0f, last_I_V = 0.0f;
static float offset_U = 2048.0f, offset_V = 2048.0f;
static float vq_cmd = 0.0f;   /* persistent limiter state — survives across cycles */

/* ============================================================ */
/* SMO + PLL state — persists across cycles                     */
/* ============================================================ */
static float smo_i_alpha = 0.0f;    /* estimated current α */
static float smo_i_beta  = 0.0f;    /* estimated current β */
static float smo_e_alpha = 0.0f;    /* filtered equivalent control ≈ back-EMF α */
static float smo_e_beta  = 0.0f;    /* filtered equivalent control ≈ back-EMF β */
static float pll_integ   = 0.0f;    /* PLL integrator (speed) */

/* ============================================================ */
/* Fast-path helpers */
/* ============================================================ */
#ifdef HOST_BUILD
static inline void foc_sync_wait(void) { }
#else
static inline void foc_sync_wait(void) {
    while (!(TIM1->SR & TIM_SR_UIF)) {}
    TIM1->SR = ~TIM_SR_UIF;
}
#endif

static inline void foc_delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * 170UL;
    while ((DWT->CYCCNT - start) < ticks) {}
}

#ifdef HOST_BUILD
static inline void get_Fast_Sin_Cos(float a, float *s, float *c) { *s = sinf(a); *c = cosf(a); }
static inline float get_Fast_Atan2(float y, float x) { return atan2f(y, x); }
#else
static inline void get_Fast_Sin_Cos(float angle_rad, float *out_sin, float *out_cos) {
    CORDIC->CSR = (6 << CORDIC_CSR_PRECISION_Pos) | CORDIC_CSR_NRES | CORDIC_CSR_NARGS | 0;
    while (angle_rad > _PI) angle_rad -= _2PI;
    while (angle_rad < -_PI) angle_rad += _2PI;
    int32_t angle_q31 = (int32_t)((angle_rad / _PI) * 2147483648.0f);
    CORDIC->WDATA = angle_q31;
    CORDIC->WDATA = 0x7FFFFFFF;
    *out_cos = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;
    *out_sin = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;
}

static inline float get_Fast_Atan2(float y, float x) {
    CORDIC->CSR = (6 << CORDIC_CSR_PRECISION_Pos) | CORDIC_CSR_NARGS | 2;
    float max_val = fmaxf(fabsf(x), fabsf(y));
    if (max_val == 0.0f) return 0.0f;
    float scale = 0.99f / max_val;
    int32_t x_q31 = (int32_t)(x * scale * 2147483648.0f);
    int32_t y_q31 = (int32_t)(y * scale * 2147483648.0f);
    CORDIC->WDATA = x_q31;
    CORDIC->WDATA = y_q31;
    return ((float)((int32_t)CORDIC->RDATA) / 2147483648.0f) * _PI;
}
#endif

static void setInverterVoltages(float v_alpha, float v_beta) {
    float Va_raw = v_alpha;
    float Vb_raw = -0.5f * v_alpha + 0.8660254f * v_beta;
    float Vc_raw = -0.5f * v_alpha - 0.8660254f * v_beta;
    float v_min = fminf(Va_raw, fminf(Vb_raw, Vc_raw));
    float v_max = fmaxf(Va_raw, fmaxf(Vb_raw, Vc_raw));
    float v_com = -(v_max + v_min) * 0.5f;

    float da = (Va_raw + v_com) / FOC_POWER_SUPPLY + 0.5f;
    float db = (Vb_raw + v_com) / FOC_POWER_SUPPLY + 0.5f;
    float dc = (Vc_raw + v_com) / FOC_POWER_SUPPLY + 0.5f;
    da = fmaxf(0.02f, fminf(da, 0.98f));
    db = fmaxf(0.02f, fminf(db, 0.98f));
    dc = fmaxf(0.02f, fminf(dc, 0.98f));

    TIM1->CCR1 = (uint16_t)(da * 4250.0f);
    TIM1->CCR2 = (uint16_t)(db * 4250.0f);
    TIM1->CCR3 = (uint16_t)(dc * 4250.0f);
    TIM1->CCER = 0xFFF;

    /* Write-back the *applied* voltage so the observer sees the
       saturated SVPWM output, not the requested vector. */
    float va_app = (da - 0.5f) * FOC_POWER_SUPPLY;
    float vb_app = (db - 0.5f) * FOC_POWER_SUPPLY;
    float vc_app = (dc - 0.5f) * FOC_POWER_SUPPLY;
    float vcom_app = (va_app + vb_app + vc_app) / 3.0f;
    float van = va_app - vcom_app;
    float vbn = vb_app - vcom_app;
    cmd_v_alpha = van;
    cmd_v_beta  = (van + 2.0f * vbn) * 0.577350269f;
}

/* ============================================================ */
/* Soft sign — reduces SMO chattering vs hard sign()            */
/* ============================================================ */
static inline float soft_sign(float x, float delta) {
    if (x >  delta) return  1.0f;
    if (x < -delta) return -1.0f;
    return x / delta;
}

/* ============================================================ */
/* Sliding-Mode Observer (SMO) + PLL angle tracker              */
/* ============================================================ */
static void execute_SMO_Observer(float I_U, float I_V) {
    /* Clarke transform */
    float I_alpha = I_U;
    float I_beta  = (I_U + 2.0f * I_V) * 0.577350269f;

    /* ---- 1. Sliding-mode current observer ---- */
    const float SMO_DELTA = 2.0f;
    const int   SUB_STEPS = 4;
    const float DT_SUB    = FOC_LOOP_DT / SUB_STEPS;

    /* [UPGRADE 2: Dynamic Sliding Gain (MESC)] */
    /* The sliding gain must always exceed the maximum possible back-EMF to force
       the observer onto the sliding surface. Instead of a fixed 8.0V (which wastes
       energy and causes chatter at low speeds), we dynamically scale it:
       K = 1.5 * Expected_BEMF + 1.0V (floor). Capped at 15.0V max. */
    float expected_bemf = FOC_LAMBDA * fabsf(estimated_speed);
    float SMO_K = 1.5f * expected_bemf + 1.0f;
    if (SMO_K > 15.0f) SMO_K = 15.0f;

    float smo_i_a = smo_i_alpha;
    float smo_i_b = smo_i_beta;
    float Z_a_sum = 0.0f;
    float Z_b_sum = 0.0f;

    for (int k = 0; k < SUB_STEPS; k++) {
        float err_a = smo_i_a - I_alpha;
        float err_b = smo_i_b - I_beta;

        float Z_a = SMO_K * soft_sign(err_a, SMO_DELTA);
        float Z_b = SMO_K * soft_sign(err_b, SMO_DELTA);

        Z_a_sum += Z_a;
        Z_b_sum += Z_b;

        /* [UPGRADE 1: BEMF Feedforward (VESC/SimpleFOC)] */
        /* We subtract the previous BEMF estimate (smo_e_alpha/beta) directly from
           the applied voltage. Because the BEMF is already pre-compensated, the
           sliding control Z is no longer burdened with tracking the full BEMF
           amplitude. Z only has to correct the *residual* error (the change in BEMF).
           This drastically reduces observer chattering. */
        smo_i_a += ((cmd_v_alpha - FOC_RS * smo_i_a - smo_e_alpha - Z_a) / FOC_LS) * DT_SUB;
        smo_i_b += ((cmd_v_beta  - FOC_RS * smo_i_b  - smo_e_beta  - Z_b) / FOC_LS) * DT_SUB;
    }

    smo_i_alpha = smo_i_a;
    smo_i_beta  = smo_i_b;

    /* ---- 2. Back-EMF extraction ---- */
    /* Since Z now only represents the *residual* error (delta BEMF), we must
       integrate it into the existing BEMF estimate rather than overwriting it. */
    smo_e_alpha += Z_a_sum / (float)SUB_STEPS;
    smo_e_beta  += Z_b_sum / (float)SUB_STEPS;

    /* ---- 3. PLL angle tracker ---- */
    float e_mag = sqrtf(smo_e_alpha * smo_e_alpha + smo_e_beta * smo_e_beta);

    if (e_mag > 0.1f) {
        float sin_th, cos_th;
        get_Fast_Sin_Cos(estimated_angle, &sin_th, &cos_th);

        float pll_err = -smo_e_alpha * cos_th - smo_e_beta * sin_th;

        /* [UPGRADE 3: Smooth Direction Flip (AM32)] */
        /* A hard IF statement causes a violent phase jump in the PLL error
           if the motor crosses 0 RPM (startup, rapid decel, or 3D reverse).
           We use a continuous soft-sign based on speed to smoothly fade
           the error direction through the zero-crossing.
           We use a 50 rad/s window for the smooth transition. */
        float dir_ref = (fabsf(estimated_speed) > 50.0f) ? estimated_speed : open_loop_speed;
        float dir_factor = soft_sign(dir_ref, 50.0f);
        pll_err *= dir_factor;

        float norm_err = pll_err / e_mag;

        float abs_speed = fabsf(estimated_speed);
        float wn = 100.0f + 0.15f * abs_speed;
        if (wn > 1500.0f) wn = 1500.0f;

        float kp = 2.0f * wn;
        float ki = wn * wn;

        pll_integ += ki * norm_err * FOC_LOOP_DT;
        if (pll_integ >  30000.0f) pll_integ =  30000.0f;
        if (pll_integ < -30000.0f) pll_integ = -30000.0f;

        estimated_speed = pll_integ + kp * norm_err;
    } else {
        pll_integ *= 0.9995f;
        estimated_speed = pll_integ;
    }

    estimated_angle += estimated_speed * FOC_LOOP_DT;
    while (estimated_angle >= _2PI) estimated_angle -= _2PI;
    while (estimated_angle <  0.0f) estimated_angle += _2PI;

    last_I_U = I_U;
    last_I_V = I_V;
}

/* ============================================================ */
/* Setup */
/* ============================================================ */
void foc_setup(void) {
    HAL_Delay(1000);

    RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
    CORDIC->CSR = (6 << CORDIC_CSR_PRECISION_Pos) | CORDIC_CSR_NRES;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    ADC1->SMPR1 |= (6 << 6);
    ADC1->SMPR2 |= (6 << 15);
    ADC2->SMPR1 |= (6 << 12);
    ADC1->JSQR  = (1 << 0) | (1 << 7) | (15 << 9) | (2 << 15);
    ADC2->JSQR  = (0 << 0) | (1 << 7) | (4 << 9);
    ADC1->CR |= ADC_CR_JADSTART;
    ADC2->CR |= ADC_CR_JADSTART;

    TIM1->CR1 = 0x20;
    TIM1->ARR = 4250;
    TIM1->CCR1 = 2125;
    TIM1->CCR2 = 2125;
    TIM1->CCR3 = 2125;
    TIM1->CCR4 = 4240;
    TIM1->BDTR = 0x80A0;
    TIM1->CR2  = 0xF3F;
    TIM1->CR2 &= ~TIM_CR2_MMS_Msk;
    TIM1->CR2 |= (0x7 << TIM_CR2_MMS_Pos);
    TIM1->EGR |= 1;
    TIM1->CR1 |= 1;

#ifdef HOST_BUILD
    offset_U = 2048.0f;
    offset_V = 2048.0f;
#else
    HAL_Delay(200);
    long sum_U = 0, sum_V = 0;
    for (int i = 0; i < 4000; i++) {
        while (!(TIM1->SR & TIM_SR_UIF)) {}
        TIM1->SR &= ~TIM_SR_UIF;
        sum_U += ADC1->JDR1;
        sum_V += ADC2->JDR1;
    }
    offset_U = (float)sum_U / 4000.0f;
    offset_V = (float)sum_V / 4000.0f;
#endif

    dshot_init();
    current_state = STATE_OFF;
    vq_cmd = 0.0f;

    /* SMO-PLL: reset observer state */
    smo_i_alpha = smo_i_beta = 0.0f;
    smo_e_alpha = smo_e_beta = 0.0f;
    pll_integ   = 0.0f;
    estimated_angle = 0.0f;
    estimated_speed = 0.0f;
    open_loop_angle = 0.0f;
    open_loop_speed = 0.0f;
}

/* ============================================================ */
/* Main loop */
/* ============================================================ */
void foc_loop(void) {
    static uint32_t tick_counter = 0;
    static uint32_t ramp_hold_timer = 0;
    static uint32_t blend_timer = 0;

    foc_sync_wait();

    float I_U = (float)((int32_t)ADC1->JDR1 - (int)offset_U) * FOC_AMPS_PER_COUNT;
    float I_V = (float)((int32_t)ADC2->JDR1 - (int)offset_V) * FOC_AMPS_PER_COUNT;

    execute_SMO_Observer(I_U, I_V);
    dshot_poll();

    switch (current_state) {
    case STATE_OFF:
        setInverterVoltages(0.0f, 0.0f);
        vq_cmd = 0.0f;

        smo_i_alpha = I_U;
        smo_i_beta  = (I_U + 2.0f * I_V) * 0.577350269f;
        smo_e_alpha = smo_e_beta = 0.0f;
        pll_integ   = 0.0f;
        estimated_speed = 0.0f;

        if (dshot.armed && dshot.throttle > 0.05f) {
            current_state = STATE_ALIGN;
            tick_counter = 0;
            open_loop_speed = 0.0f;
            open_loop_angle = 0.0f;
            estimated_angle = 0.0f;
            ramp_hold_timer = 0;
            blend_timer = 0;
        }
        break;

    case STATE_ALIGN: {
        float align_v = (float)tick_counter / 10000.0f * 0.4f;
        if (align_v > 0.4f) align_v = 0.4f;
        cmd_v_alpha = align_v; cmd_v_beta = 0.0f;
        setInverterVoltages(cmd_v_alpha, cmd_v_beta);

        if (tick_counter > 2000) {
            tick_counter = 0;
            current_state = STATE_OPEN_LOOP_RAMP;
            open_loop_angle = 0.0f;
            estimated_angle = 0.0f;
            pll_integ = 0.0f;
        }
        if (dshot.throttle <= 0.02f) current_state = STATE_OFF;
        break;
    }

    case STATE_OPEN_LOOP_RAMP: {
        float ramp_target = 300.0f + 700.0f * dshot.throttle;
        float ramp_rate   = 3000.0f + 5000.0f * dshot.throttle;

        if (open_loop_speed < ramp_target) {
            open_loop_speed += ramp_rate * FOC_LOOP_DT;
            ramp_hold_timer = 0;
        } else {
            open_loop_speed = ramp_target;
            ramp_hold_timer++;
        }
        open_loop_angle += open_loop_speed * FOC_LOOP_DT;
        if (open_loop_angle > _2PI) open_loop_angle -= _2PI;

        float sin_ol, cos_ol;
        get_Fast_Sin_Cos(open_loop_angle, &sin_ol, &cos_ol);
        float v_mag = open_loop_speed * 0.0015f;
        cmd_v_alpha = -v_mag * sin_ol;
        cmd_v_beta  =  v_mag * cos_ol;
        setInverterVoltages(cmd_v_alpha, cmd_v_beta);

        if (ramp_hold_timer > 1500) {
            current_state = STATE_FOC_BLEND;
            ramp_hold_timer = 0;
            blend_timer = 0;
            vq_cmd = v_mag;
            pll_integ = open_loop_speed;
        }
        if (dshot.throttle <= 0.02f) current_state = STATE_OFF;
        break;
    }

    case STATE_FOC_BLEND: {
        blend_timer++;
        uint32_t blend_cycles = 3000u + (uint32_t)(2000.0f * dshot.throttle);
        float blend_factor = (float)blend_timer / (float)blend_cycles;

        float MIN_VQ = 0.75f, MAX_VQ = 14.0f;
        float target_vq = MIN_VQ + dshot.throttle * (MAX_VQ - MIN_VQ);

        open_loop_angle += open_loop_speed * FOC_LOOP_DT;
        if (open_loop_angle > _2PI) open_loop_angle -= _2PI;

        float angle_diff = estimated_angle - open_loop_angle;
        if (angle_diff > _PI)  angle_diff -= _2PI;
        if (angle_diff < -_PI) angle_diff += _2PI;

        float blended_angle = open_loop_angle + angle_diff * blend_factor;
        if (blended_angle >= _2PI) blended_angle -= _2PI;
        if (blended_angle < 0.0f)  blended_angle += _2PI;

        cmd_vq_throttle = MIN_VQ + ((target_vq - MIN_VQ) * blend_factor);

        float sin_cl, cos_cl;
        get_Fast_Sin_Cos(blended_angle, &sin_cl, &cos_cl);

        float I_alpha_bl = I_U;
        float I_beta_bl  = (I_U + 2.0f * I_V) * 0.577350269f;
        float iq_bl = I_alpha_bl * sin_cl - I_beta_bl * cos_cl;
        if (fabsf(iq_bl) > 20.0f) {
            cmd_vq_throttle *= (20.0f / fabsf(iq_bl));
        }

        cmd_v_alpha = -cmd_vq_throttle * sin_cl;
        cmd_v_beta  =  cmd_vq_throttle * cos_cl;
        setInverterVoltages(cmd_v_alpha, cmd_v_beta);

        if (blend_timer >= blend_cycles) {
            current_state = STATE_CLOSED_LOOP_FOC;
            blend_timer = 0;
            vq_cmd = cmd_vq_throttle;
        }
        if (dshot.throttle <= 0.02f) current_state = STATE_OFF;
        break;
    }

    case STATE_CLOSED_LOOP_FOC: {
        float MIN_VQ = 0.75f, MAX_VQ = 14.0f;
        float target_vq = MIN_VQ + dshot.throttle * (MAX_VQ - MIN_VQ);

        float phase_advance = estimated_speed * 40e-6f;
        float advanced_angle = estimated_angle + phase_advance;
        if (advanced_angle > _2PI) advanced_angle -= _2PI;
        if (advanced_angle < 0.0f) advanced_angle += _2PI;

        float sin_cl, cos_cl;
        get_Fast_Sin_Cos(advanced_angle, &sin_cl, &cos_cl);

        float I_alpha = I_U;
        float I_beta  = (I_U + 2.0f * I_V) * 0.577350269f;
        float iq = I_alpha * sin_cl - I_beta * cos_cl;
        float vd_ff = -estimated_speed * FOC_LS * iq;

        float i_total = sqrtf(I_alpha * I_alpha + I_beta * I_beta);

        float abs_w = fabsf(estimated_speed);
        float i_limit;
        if (abs_w < 2000.0f) {
            i_limit = 25.0f;
        } else if (abs_w > 6000.0f) {
            i_limit = 45.0f;
        } else {
            i_limit = 25.0f + 20.0f * (abs_w - 2000.0f) / 4000.0f;
        }

        if (i_total > i_limit) {
            vq_cmd *= (i_limit / i_total);
            if (vq_cmd < 0.1f) vq_cmd = 0.1f;
        } else {
            if (vq_cmd < target_vq) {
                vq_cmd += 0.05f;
                if (vq_cmd > target_vq) vq_cmd = target_vq;
            } else if (vq_cmd > target_vq) {
                vq_cmd = target_vq;
            }
        }

        cmd_vq_throttle = vq_cmd;

        float v_mag = sqrtf(vd_ff * vd_ff + cmd_vq_throttle * cmd_vq_throttle);
        float v_max = FOC_POWER_SUPPLY / 1.7320508f;
        float v_scale = (v_mag > v_max) ? (v_max / v_mag) : 1.0f;

        cmd_v_alpha = (vd_ff * v_scale * cos_cl) - (cmd_vq_throttle * v_scale * sin_cl);
        cmd_v_beta  = (vd_ff * v_scale * sin_cl) + (cmd_vq_throttle * v_scale * cos_cl);

        setInverterVoltages(cmd_v_alpha, cmd_v_beta);

        if (dshot.throttle <= 0.02f) {
            current_state = STATE_OFF;
            vq_cmd = 0.0f;
        }
        break;
    }
    }

    dshot_set_erpm((uint32_t)(estimated_speed * 60.0f / (2.0f * _PI)));
    tick_counter++;
}

void foc_set_adc_offsets(float u, float v) { offset_U = u; offset_V = v; }

DriveState foc_get_state(void)         { return current_state; }
float foc_get_estimated_speed(void)    { return estimated_speed; }
float foc_get_estimated_angle(void)    { return estimated_angle; }
float foc_get_phase_u_current(void)    { return last_I_U; }
float foc_get_phase_v_current(void)    { return last_I_V; }
float foc_get_v_alpha(void)            { return cmd_v_alpha; }
float foc_get_v_beta(void)             { return cmd_v_beta; }