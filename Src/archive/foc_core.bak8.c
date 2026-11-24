/*
 * foc_core.c — Production-Grade FOC engine (48 kHz Optimized)
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

static DriveState current_state = STATE_OFF;
static float cmd_v_alpha = 0.0f, cmd_v_beta = 0.0f;
static float estimated_angle = 0.0f, estimated_speed = 0.0f;
static float open_loop_speed = 0.0f, open_loop_angle = 0.0f;

static float last_I_U = 0.0f, last_I_V = 0.0f;
static float offset_U = 2048.0f, offset_V = 2048.0f;

static float vq_pi_integ = 0.0f;
static float vd_pi_integ = 0.0f;

static float smo_i_alpha = 0.0f;
static float smo_i_beta  = 0.0f;
static float smo_e_alpha = 0.0f;
static float smo_e_beta  = 0.0f;
static float pll_integ   = 0.0f;

#ifdef HOST_BUILD
static inline void foc_sync_wait(void) { }
#else
static inline void foc_sync_wait(void) {
    while (!(TIM1->SR & TIM_SR_UIF)) {}
    TIM1->SR = ~TIM_SR_UIF;
}
#endif

#ifdef HOST_BUILD
static inline void get_Fast_Sin_Cos(float a, float *s, float *c) { *s = sinf(a); *c = cosf(a); }
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
#endif

static inline float soft_sign(float x, float delta) {
    if (x >  delta) return  1.0f;
    if (x < -delta) return -1.0f;
    return x / delta;
}

static void setInverterVoltages(float v_alpha, float v_beta) {
    if (current_state != STATE_OFF) {
        float I_a = last_I_U;
        float I_b = (last_I_U + 2.0f * last_I_V) * INV_SQRT3;
        float sign_a = soft_sign(I_a, 1.5f);
        float sign_b = soft_sign(I_b, 1.5f);

        v_alpha += 0.25f * sign_a;
        v_beta  += 0.25f * sign_b;
    }

    float Va_raw = v_alpha;
    float Vb_raw = -0.5f * v_alpha + 0.8660254f * v_beta;
    float Vc_raw = -0.5f * v_alpha - 0.8660254f * v_beta;

    /* FIX: Inline ternary min/max instead of fminf/fmaxf */
    float v_min = (Va_raw < Vb_raw) ? ((Va_raw < Vc_raw) ? Va_raw : Vc_raw) : ((Vb_raw < Vc_raw) ? Vb_raw : Vc_raw);
    float v_max = (Va_raw > Vb_raw) ? ((Va_raw > Vc_raw) ? Va_raw : Vc_raw) : ((Vb_raw > Vc_raw) ? Vb_raw : Vc_raw);
    float v_com = -(v_max + v_min) * 0.5f;

    /* FIX: Multiplication by reciprocal instead of division */
    float da = (Va_raw + v_com) * INV_SUPPLY + 0.5f;
    float db = (Vb_raw + v_com) * INV_SUPPLY + 0.5f;
    float dc = (Vc_raw + v_com) * INV_SUPPLY + 0.5f;

    /* FIX: Inline ternary clamps */
    da = (da < 0.02f) ? 0.02f : ((da > 0.98f) ? 0.98f : da);
    db = (db < 0.02f) ? 0.02f : ((db > 0.98f) ? 0.98f : db);
    dc = (dc < 0.02f) ? 0.02f : ((dc > 0.98f) ? 0.98f : dc);

    /* FIX: Changed 4250.0f to 1770.0f for 48 kHz */
    TIM1->CCR1 = (uint16_t)(da * TIM1_ARR);
    TIM1->CCR2 = (uint16_t)(db * TIM1_ARR);
    TIM1->CCR3 = (uint16_t)(dc * TIM1_ARR);
    TIM1->CCER = 0xFFF;

    float va_app = (da - 0.5f) * FOC_POWER_SUPPLY;
    float vb_app = (db - 0.5f) * FOC_POWER_SUPPLY;
    float vc_app = (dc - 0.5f) * FOC_POWER_SUPPLY;

    /* FIX: Multiply by INV_3_0 */
    float vcom_app = (va_app + vb_app + vc_app) * INV_3_0;
    float van = va_app - vcom_app;
    float vbn = vb_app - vcom_app;
    cmd_v_alpha = van;
    cmd_v_beta  = (van + 2.0f * vbn) * INV_SQRT3;
}

static void execute_SMO_Observer(float I_U, float I_V) {
    float I_alpha = I_U;
    float I_beta  = (I_U + 2.0f * I_V) * INV_SQRT3;

    const float SMO_DELTA = 2.0f;
    const int   SUB_STEPS = 4;
    const float DT_SUB    = FOC_LOOP_DT / SUB_STEPS;
    const float SMO_K = 15.0f;

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

        /* FIX: Replace / FOC_LS with * INV_LS */
        smo_i_a += ((cmd_v_alpha - FOC_RS * smo_i_a - smo_e_alpha - Z_a) * INV_LS) * DT_SUB;
        smo_i_b += ((cmd_v_beta  - FOC_RS * smo_i_b  - smo_e_beta  - Z_b) * INV_LS) * DT_SUB;
    }

    smo_i_alpha = smo_i_a;
    smo_i_beta  = smo_i_b;

    float abs_speed = (estimated_speed < 0.0f) ? -estimated_speed : estimated_speed;
    float lpf_wc = 500.0f + 2.0f * abs_speed;
    if (lpf_wc > 25000.0f) lpf_wc = 25000.0f;
    float lpf_a = lpf_wc * FOC_LOOP_DT;
    if (lpf_a > 1.0f) lpf_a = 1.0f;

    smo_e_alpha += ((Z_a_sum / (float)SUB_STEPS) - smo_e_alpha) * lpf_a;
    smo_e_beta  += ((Z_b_sum / (float)SUB_STEPS) - smo_e_beta) * lpf_a;

    float e_mag = sqrtf(smo_e_alpha * smo_e_alpha + smo_e_beta * smo_e_beta);

    if (e_mag > 0.1f) {
        float sin_th, cos_th;
        get_Fast_Sin_Cos(estimated_angle, &sin_th, &cos_th);

        float pll_err = -smo_e_alpha * cos_th - smo_e_beta * sin_th;
        float dir_ref = (abs_speed > 50.0f) ? estimated_speed : open_loop_speed;
        float dir_factor = soft_sign(dir_ref, 50.0f);
        pll_err *= dir_factor;

        float norm_err = pll_err / e_mag;

        float wn = 200.0f + 0.15f * abs_speed;
        if (wn > 3000.0f) wn = 3000.0f;

        float kp = 2.0f * wn;
        float ki = wn * wn;

        pll_integ += ki * norm_err * FOC_LOOP_DT;
        /* FIX: Inline ternary clamp */
        pll_integ = (pll_integ < -25000.0f) ? -25000.0f : ((pll_integ > 25000.0f) ? 25000.0f : pll_integ);

        estimated_speed = pll_integ + kp * norm_err;
    } else {
        pll_integ *= 0.9995f;
        estimated_speed = pll_integ;
    }

    /* FIX: Inline ternary clamp */
    estimated_speed = (estimated_speed < -25000.0f) ? -25000.0f : ((estimated_speed > 25000.0f) ? 25000.0f : estimated_speed);

    estimated_angle += estimated_speed * FOC_LOOP_DT;
    while (estimated_angle >= _2PI) estimated_angle -= _2PI;
    while (estimated_angle <  0.0f) estimated_angle += _2PI;

    last_I_U = I_U;
    last_I_V = I_V;
}

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
    /* FIX: ARR set to 1770 for 48 kHz */
    TIM1->ARR = (uint16_t)TIM1_ARR;
    TIM1->CCR1 = (uint16_t)(TIM1_ARR / 2);
    TIM1->CCR2 = (uint16_t)(TIM1_ARR / 2);
    TIM1->CCR3 = (uint16_t)(TIM1_ARR / 2);
    TIM1->CCR4 = (uint16_t)(TIM1_ARR - 10);
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
    vq_pi_integ = 0.0f;
    vd_pi_integ = 0.0f;

    smo_i_alpha = smo_i_beta = 0.0f;
    smo_e_alpha = smo_e_beta = 0.0f;
    pll_integ   = 0.0f;
    estimated_angle = 0.0f;
    estimated_speed = 0.0f;
    open_loop_angle = 0.0f;
    open_loop_speed = 0.0f;
}

void foc_loop(void) {
    static uint32_t tick_counter = 0;
    static uint32_t blend_timer = 0;
    static uint32_t kick_hold_timer = 0;

    foc_sync_wait();

    float I_U = (float)((int32_t)ADC1->JDR1 - (int)offset_U) * FOC_AMPS_PER_COUNT;
    float I_V = (float)((int32_t)ADC2->JDR1 - (int)offset_V) * FOC_AMPS_PER_COUNT;

    execute_SMO_Observer(I_U, I_V);
    dshot_poll();

    switch (current_state) {
    case STATE_OFF:
        setInverterVoltages(0.0f, 0.0f);
        vq_pi_integ = 0.0f;
        vd_pi_integ = 0.0f;

        smo_i_alpha = I_U;
        smo_i_beta  = (I_U + 2.0f * I_V) * INV_SQRT3;
        smo_e_alpha = smo_e_beta = 0.0f;
        pll_integ   = 0.0f;
        estimated_speed = 0.0f;

        if (dshot.armed && dshot.throttle > 0.05f) {
            tick_counter = 0;
            open_loop_speed = 0.0f;
            open_loop_angle = 0.0f;
            estimated_angle = 0.0f;
            blend_timer = 0;
            kick_hold_timer = 0;
            current_state = STATE_ALIGN;
        }
        break;

    case STATE_ALIGN: {
        float align_v = (float)tick_counter / 1000.0f * 0.4f;
        if (align_v > 0.4f) align_v = 0.4f;

        cmd_v_alpha = 0.0f;
        cmd_v_beta  = align_v;
        setInverterVoltages(cmd_v_alpha, cmd_v_beta);

        if (tick_counter > 1000) {
            tick_counter = 0;
            current_state = STATE_KICK_START;
        }
        if (dshot.throttle <= 0.02f) current_state = STATE_OFF;
        break;
    }

    case STATE_KICK_START: {
        float kick_target = 1400.0f + 4000.0f * dshot.throttle;
        float kick_rate = 8000.0f + 10000.0f * dshot.throttle;

        if (open_loop_speed < kick_target) {
            open_loop_speed += kick_rate * FOC_LOOP_DT;
            kick_hold_timer = 0;
        } else {
            open_loop_speed = kick_target;
            kick_hold_timer++;
        }

        open_loop_angle += open_loop_speed * FOC_LOOP_DT;
        if (open_loop_angle > _2PI) open_loop_angle -= _2PI;

        float sin_ol, cos_ol;
        get_Fast_Sin_Cos(open_loop_angle, &sin_ol, &cos_ol);

        float v_mag = (open_loop_speed * FOC_LAMBDA) + 1.0f;

        cmd_v_alpha = -v_mag * sin_ol;
        cmd_v_beta  =  v_mag * cos_ol;
        setInverterVoltages(cmd_v_alpha, cmd_v_beta);

        if (kick_hold_timer > 2000) {
            current_state = STATE_FOC_BLEND;
            blend_timer = 0;
            pll_integ = open_loop_speed;
            vq_pi_integ = open_loop_speed * FOC_LAMBDA;
            vd_pi_integ = 0.0f;
        }
        if (dshot.throttle <= 0.02f) current_state = STATE_OFF;
        break;
    }

    case STATE_FOC_BLEND: {
        blend_timer++;
        uint32_t blend_cycles = 4000u + (uint32_t)(8000.0f * dshot.throttle);
        float blend_factor = (float)blend_timer / (float)blend_cycles;
        if (blend_factor > 1.0f) blend_factor = 1.0f;

        open_loop_angle += open_loop_speed * FOC_LOOP_DT;
        if (open_loop_angle > _2PI) open_loop_angle -= _2PI;

        float angle_diff = estimated_angle - open_loop_angle;
        if (angle_diff > _PI)  angle_diff -= _2PI;
        if (angle_diff < -_PI) angle_diff += _2PI;

        float blended_angle = open_loop_angle + angle_diff * blend_factor;
        if (blended_angle >= _2PI) blended_angle -= _2PI;
        if (blended_angle < 0.0f)  blended_angle += _2PI;

        float sin_cl, cos_cl;
        get_Fast_Sin_Cos(blended_angle, &sin_cl, &cos_cl);

        float I_alpha = I_U;
        float I_beta  = (I_U + 2.0f * I_V) * INV_SQRT3;

        float Id_meas =  I_alpha * cos_cl + I_beta * sin_cl;
        float Iq_meas = -I_alpha * sin_cl + I_beta * cos_cl;

        float abs_w = (estimated_speed < 0.0f) ? -estimated_speed : estimated_speed;
        float i_limit;
        if (abs_w < 2000.0f) i_limit = 25.0f;
        else if (abs_w > 6000.0f) i_limit = 40.0f; // CHANGED from 45.0f (AD8418A limit)
        else i_limit = 25.0f + 20.0f * (abs_w - 2000.0f) * INV_4000;

        float Iq_target = (dshot.throttle * i_limit) * blend_factor;
        float Id_target = 0.0f;

        float Iq_err = Iq_target - Iq_meas;
        float Id_err = Id_target - Id_meas;

        const float KP_CURR = 0.1f;
        const float KI_CURR = 200.0f;
        const float V_MAX = FOC_POWER_SUPPLY * INV_SQRT3;

        vq_pi_integ += KI_CURR * Iq_err * FOC_LOOP_DT;
        vq_pi_integ = (vq_pi_integ < -V_MAX) ? -V_MAX : ((vq_pi_integ > V_MAX) ? V_MAX : vq_pi_integ);
        float Vq_pi = KP_CURR * Iq_err + vq_pi_integ;

        vd_pi_integ += KI_CURR * Id_err * FOC_LOOP_DT;
        vd_pi_integ = (vd_pi_integ < -V_MAX) ? -V_MAX : ((vd_pi_integ > V_MAX) ? V_MAX : vd_pi_integ);
        float Vd_pi = KP_CURR * Id_err + vd_pi_integ;

        float Vd_ff = -estimated_speed * FOC_LS * Iq_meas;
        float Vq_ff = estimated_speed * FOC_LS * Id_meas + estimated_speed * FOC_LAMBDA;

        float Vd = Vd_pi + Vd_ff;
        float Vq = Vq_pi + Vq_ff;

        float v_mag = sqrtf(Vd * Vd + Vq * Vq);
        if (v_mag > V_MAX) {
            float scale = V_MAX / v_mag;
            Vd *= scale;
            Vq *= scale;
            vd_pi_integ = Vd - Vd_ff - KP_CURR * Id_err;
            vq_pi_integ = Vq - Vq_ff - KP_CURR * Iq_err;
        }

        cmd_v_alpha = (Vd * cos_cl) - (Vq * sin_cl);
        cmd_v_beta  = (Vd * sin_cl) + (Vq * cos_cl);

        setInverterVoltages(cmd_v_alpha, cmd_v_beta);

        if (blend_timer >= blend_cycles) {
            current_state = STATE_CLOSED_LOOP_FOC;
            blend_timer = 0;
        }
        if (dshot.throttle <= 0.02f) current_state = STATE_OFF;
        break;
    }

    case STATE_CLOSED_LOOP_FOC: {
        float phase_advance = estimated_speed * 40e-6f;
        float advanced_angle = estimated_angle + phase_advance;
        if (advanced_angle > _2PI) advanced_angle -= _2PI;
        if (advanced_angle < 0.0f) advanced_angle += _2PI;

        float sin_cl, cos_cl;
        get_Fast_Sin_Cos(advanced_angle, &sin_cl, &cos_cl);

        float I_alpha = I_U;
        float I_beta  = (I_U + 2.0f * I_V) * INV_SQRT3;

        float Id_meas =  I_alpha * cos_cl + I_beta * sin_cl;
        float Iq_meas = -I_alpha * sin_cl + I_beta * cos_cl;

        float abs_w = (estimated_speed < 0.0f) ? -estimated_speed : estimated_speed;
        float i_limit;
        if (abs_w < 2000.0f) i_limit = 25.0f;
        else if (abs_w > 6000.0f) i_limit = 45.0f;
        else i_limit = 25.0f + 20.0f * (abs_w - 2000.0f) * INV_4000;

        float Iq_target = dshot.throttle * i_limit;
        float Id_target = 0.0f;

        float Iq_err = Iq_target - Iq_meas;
        float Id_err = Id_target - Id_meas;

        const float KP_CURR = 0.1f;
        const float KI_CURR = 200.0f;
        const float V_MAX = FOC_POWER_SUPPLY * INV_SQRT3;

        vq_pi_integ += KI_CURR * Iq_err * FOC_LOOP_DT;
        vq_pi_integ = (vq_pi_integ < -V_MAX) ? -V_MAX : ((vq_pi_integ > V_MAX) ? V_MAX : vq_pi_integ);
        float Vq_pi = KP_CURR * Iq_err + vq_pi_integ;

        vd_pi_integ += KI_CURR * Id_err * FOC_LOOP_DT;
        vd_pi_integ = (vd_pi_integ < -V_MAX) ? -V_MAX : ((vd_pi_integ > V_MAX) ? V_MAX : vd_pi_integ);
        float Vd_pi = KP_CURR * Id_err + vd_pi_integ;

        float Vd_ff = -estimated_speed * FOC_LS * Iq_meas;
        float Vq_ff = estimated_speed * FOC_LS * Id_meas + estimated_speed * FOC_LAMBDA;

        float Vd = Vd_pi + Vd_ff;
        float Vq = Vq_pi + Vq_ff;

        float v_mag = sqrtf(Vd * Vd + Vq * Vq);
        if (v_mag > V_MAX) {
            float scale = V_MAX / v_mag;
            Vd *= scale;
            Vq *= scale;
            vd_pi_integ = Vd - Vd_ff - KP_CURR * Id_err;
            vq_pi_integ = Vq - Vq_ff - KP_CURR * Iq_err;
        }

        cmd_v_alpha = (Vd * cos_cl) - (Vq * sin_cl);
        cmd_v_beta  = (Vd * sin_cl) + (Vq * cos_cl);

        setInverterVoltages(cmd_v_alpha, cmd_v_beta);

        if (dshot.throttle <= 0.02f) {
            current_state = STATE_OFF;
            vq_pi_integ = 0.0f;
            vd_pi_integ = 0.0f;
        }
        break;
    }
    }

    dshot_set_erpm((uint32_t)(estimated_speed * 60.0f * 0.15915494f)); // Replaced / (2*PI) with * (1/(2*PI))
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
float foc_get_pll_integ(void)          { return pll_integ; }