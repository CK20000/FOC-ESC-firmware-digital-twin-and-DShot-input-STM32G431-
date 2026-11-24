/*
 * foc_core.c — Production-Grade FOC engine (I-f Startup)
 * Optimized: CCM SRAM, Pipelined CORDIC, Integer Offsets, Multiply-only Soft Sign, Fast RSQRT.
 */
#include "foc_core.h"
#include "foc_dshot.h"
#ifdef HOST_BUILD
#include "virtual_stm32.h"
#else
#include "stm32g4xx.h"
#endif
#include <math.h>
#include <string.h>

#define _PI  3.14159265359f
#define _2PI 6.28318530718f

// Precomputed 2^31 / PI for fast Q31 conversion
#define INV_PI_2_31 683565275.0f

// Toggle Fast Inverse Square Root (saves ~16 cycles per call)
#define USE_FAST_RSQRT 1
#define BENCHMARK_MODE 0

static DriveState current_state = STATE_OFF;
static float cmd_v_alpha = 0.0f, cmd_v_beta = 0.0f;
static float estimated_angle = 0.0f, estimated_speed = 0.0f;
static float open_loop_speed = 0.0f, open_loop_angle = 0.0f;

static float last_I_U = 0.0f, last_I_V = 0.0f;
static int32_t offset_U = 2048, offset_V = 2048; // Changed to int32_t to save cycles

static float vq_pi_integ = 0.0f;
static float vd_pi_integ = 0.0f;

static float smo_i_alpha = 0.0f;
static float smo_i_beta  = 0.0f;
static float smo_e_alpha = 0.0f;
static float smo_e_beta  = 0.0f;
static float pll_integ   = 0.0f;
volatile DebugInfo foc_debug = {0};
static uint32_t stall_timer = 0;
static uint32_t fault_retry_timer = 0;  // Counts loops in fault state
static uint32_t fault_retry_count = 0;  // Counts how many times we tried to recover
float v_bus_measured = 25.2f;

/* --- OPTIMIZATION: CCM SRAM SECTION --- */
#ifdef HOST_BUILD
    #define FOC_CCM
#else
    #define FOC_CCM __attribute__((section(".ccmram")))
#endif

#ifdef HOST_BUILD
static inline void foc_sync_wait(void) { }
#else
static inline void foc_sync_wait(void) {
    while (!(TIM1->SR & TIM_SR_UIF)) {}
    TIM1->SR = ~TIM_SR_UIF;
}
#endif

/* --- OPTIMIZATION: FAST RSQRT --- */
static inline float fast_rsqrt(float x) {
    uint32_t i; float y = x;
    memcpy(&i, &y, 4);
    i = 0x5f3759dfu - (i >> 1);
    memcpy(&y, &i, 4);
    return y * (1.5f - 0.5f * x * y * y);   // one Newton step
}

/* --- OPTIMIZATION: MULTIPLY-BASED SOFT SIGN --- */
static inline float soft_sign_opt(float x, float delta, float inv_delta) {
    if (x >  delta) return  1.0f;
    if (x < -delta) return -1.0f;
    return x * inv_delta;
}

/* --- OPTIMIZATION: SPLIT CORDIC --- */
static inline void cordic_start(float angle_rad) {
#ifdef HOST_BUILD
    // SIL stub
    (void)angle_rad;
#else
    CORDIC->CSR = (6 << CORDIC_CSR_PRECISION_Pos) | CORDIC_CSR_NRES | CORDIC_CSR_NARGS | 0;
    while (angle_rad > _PI) angle_rad -= _2PI;
    while (angle_rad < -_PI) angle_rad += _2PI;
    int32_t angle_q31 = (int32_t)(angle_rad * INV_PI_2_31);
    CORDIC->WDATA = angle_q31;
    CORDIC->WDATA = 0x7FFFFFFF;
#endif
}

static inline void cordic_read(float angle_rad, float *out_sin, float *out_cos) {
#ifdef HOST_BUILD
    *out_sin = sinf(angle_rad);
    *out_cos = cosf(angle_rad);
#else
    *out_cos = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;
    *out_sin = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;
#endif
}

FOC_CCM static void setInverterVoltages(float v_alpha, float v_beta) {
    if (current_state != STATE_OFF) {
        float I_a = last_I_U;
        float I_b = -(last_I_U + 2.0f * last_I_V) * INV_SQRT3;

        // Use optimized soft sign (multiply instead of divide)
        float sign_a = soft_sign_opt(I_a, 1.5f, 0.66666667f);
        float sign_b = soft_sign_opt(I_b, 1.5f, 0.66666667f);

        v_alpha += 0.25f * sign_a;
        v_beta  += 0.25f * sign_b;
    }

    float Va_raw = v_alpha;
    float Vb_raw = -0.5f * v_alpha + 0.8660254f * v_beta;
    float Vc_raw = -0.5f * v_alpha - 0.8660254f * v_beta;

    float v_min = (Va_raw < Vb_raw) ? ((Va_raw < Vc_raw) ? Va_raw : Vc_raw) : ((Vb_raw < Vc_raw) ? Vb_raw : Vc_raw);
    float v_max = (Va_raw > Vb_raw) ? ((Va_raw > Vc_raw) ? Va_raw : Vc_raw) : ((Vb_raw > Vc_raw) ? Vb_raw : Vc_raw);
    float v_com = -(v_max + v_min) * 0.5f;

    float inv_vbus = 1.0f / v_bus_measured;
    float da = (Va_raw + v_com) * inv_vbus + 0.5f;
    float db = (Vb_raw + v_com) * inv_vbus + 0.5f;
    float dc = (Vc_raw + v_com) * inv_vbus + 0.5f;

    da = (da < 0.02f) ? 0.02f : ((da > 0.98f) ? 0.98f : da);
    db = (db < 0.02f) ? 0.02f : ((db > 0.98f) ? 0.98f : db);
    dc = (dc < 0.02f) ? 0.02f : ((dc > 0.98f) ? 0.98f : dc);

    TIM1->CCR1 = (uint16_t)(da * TIM1_ARR);
    TIM1->CCR2 = (uint16_t)(db * TIM1_ARR);
    TIM1->CCR3 = (uint16_t)(dc * TIM1_ARR);
    TIM1->CCER = 0xFFF;

    float va_app = (da - 0.5f) * v_bus_measured;
    float vb_app = (db - 0.5f) * v_bus_measured;
    float vc_app = (dc - 0.5f) * v_bus_measured;

    float vcom_app = (va_app + vb_app + vc_app) * INV_3_0;
    float van = va_app - vcom_app;
    float vbn = vb_app - vcom_app;
    cmd_v_alpha = van;
    cmd_v_beta  = (van + 2.0f * vbn) * INV_SQRT3;
}

FOC_CCM static void execute_SMO_Observer(float I_U, float I_V) {
    float I_alpha = I_U;
    float I_beta  = -(I_U + 2.0f * I_V) * INV_SQRT3;

    // 1. START CORDIC FIRST (Non-blocking)
    float old_angle = estimated_angle;
    cordic_start(old_angle);

    const float SMO_DELTA = 2.0f;
    const float INV_DELTA = 0.5f; // 1.0f / 2.0f
    const int   SUB_STEPS = 4;
    const float DT_SUB    = FOC_LOOP_DT / SUB_STEPS;
    const float SMO_K = 15.0f;

    float smo_i_a = smo_i_alpha;
    float smo_i_b = smo_i_beta;
    float Z_a_sum = 0.0f;
    float Z_b_sum = 0.0f;

    // 2. DO THE HEAVY SMO MATH WHILE CORDIC CALCULATES
    for (int k = 0; k < SUB_STEPS; k++) {
        float err_a = smo_i_a - I_alpha;
        float err_b = smo_i_b - I_beta;

        float Z_a = SMO_K * soft_sign_opt(err_a, SMO_DELTA, INV_DELTA);
        float Z_b = SMO_K * soft_sign_opt(err_b, SMO_DELTA, INV_DELTA);

        Z_a_sum += Z_a;
        Z_b_sum += Z_b;

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

    // 3. READ CORDIC RESULT
    float sin_th, cos_th;
    cordic_read(old_angle, &sin_th, &cos_th);

    float e_sq = smo_e_alpha * smo_e_alpha + smo_e_beta * smo_e_beta;

    // 4. FAST SQRT / NORMALIZATION
    if (e_sq > 0.01f) { // 0.1^2
        float pll_err = -smo_e_alpha * cos_th - smo_e_beta * sin_th;
        float dir_ref = (abs_speed > 50.0f) ? estimated_speed : open_loop_speed;
        float dir_factor = soft_sign_opt(dir_ref, 50.0f, 0.02f); // 1.0f / 50.0f
        pll_err *= dir_factor;

#if USE_FAST_RSQRT
        float norm_err = pll_err * fast_rsqrt(e_sq);
#else
        float norm_err = pll_err / sqrtf(e_sq);
#endif

        float wn = 200.0f + 0.15f * abs_speed;
        float kp = 2.0f * wn;
        float ki = wn * wn;

        pll_integ += ki * norm_err * FOC_LOOP_DT;
        pll_integ = (pll_integ < -25000.0f) ? -25000.0f : ((pll_integ > 25000.0f) ? 25000.0f : pll_integ);

        estimated_speed = pll_integ + kp * norm_err;
    } else {
        pll_integ *= 0.9995f;
        estimated_speed = pll_integ;
    }

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
    offset_U = 2048;
    offset_V = 2048;
#else
    HAL_Delay(200);
    long sum_U = 0, sum_V = 0;
    for (int i = 0; i < 4000; i++) {
        while (!(TIM1->SR & TIM_SR_UIF)) {}
        TIM1->SR &= ~TIM_SR_UIF;
        sum_U += ADC1->JDR1;
        sum_V += ADC2->JDR1;
    }
    offset_U = (int32_t)(sum_U / 4000);
    offset_V = (int32_t)(sum_V / 4000);
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

    // --- HACK: FORCE CLOSED LOOP FOR BENCHMARKING ---
    current_state = STATE_OFF;
}

FOC_CCM void foc_loop(void) {
    static uint32_t tick_counter = 0;
    static uint32_t blend_timer = 0;
    static uint32_t vbus_timer = 0;

    foc_sync_wait();

    // Update VBUS every 48 cycles (1 kHz sampling)
    if (++vbus_timer >= 48) {
        float vbus_counts = (float)ADC1->JDR3;
        v_bus_measured = vbus_counts * (3.3f / 4095.0f) * 11.0f;

        // Safety clamp
        if (v_bus_measured < 10.0f) v_bus_measured = 10.0f;
        if (v_bus_measured > 30.0f) v_bus_measured = 30.0f;

        vbus_timer = 0;
    }

    #if BENCHMARK_MODE
    // FORCE CLOSED LOOP FOR BENCHMARKING
    current_state = STATE_CLOSED_LOOP_FOC;
    dshot.throttle = 0.8f;
    dshot.armed = true;
    estimated_speed = 6000.0f; // Force max current limit
    estimated_angle += 0.1f;   // Fake rotation so trig math runs
    #endif

    uint32_t start_cycles = DWT->CYCCNT;

    // Integer subtraction before float conversion
    float I_U = (float)((int32_t)ADC1->JDR1 - offset_U) * FOC_AMPS_PER_COUNT;
    float I_V = (float)((int32_t)ADC2->JDR1 - offset_V) * FOC_AMPS_PER_COUNT;

    execute_SMO_Observer(I_U, I_V);
    dshot_poll();

    switch (current_state) {

    case STATE_ALIGN:
        break;
    case STATE_FAULT: {
        //setInverterVoltages(0.0f, 0.0f);
        TIM1->CCER = 0x0000;
        vq_pi_integ = 0.0f;
        vd_pi_integ = 0.0f;

        fault_retry_timer++;

        // Wait exactly 25ms (1200 loops at 48kHz) for current to decay
        if (fault_retry_timer >= 1200) {
            fault_retry_timer = 0;

            // HARD STALL CONDITION: If we've failed 3 times, stay here.
            if (fault_retry_count >= 3) {
                static uint32_t lockout_timer = 0;
                lockout_timer++;
                if (lockout_timer > 40) { // 1 second
                    fault_retry_count = 0;
                    lockout_timer = 0; // <--- CRITICAL: Reset the timer
                    current_state = STATE_OFF;
                }
                break;
            }

            // Increment the retry counter because we are attempting a recovery
            fault_retry_count++;

            if (fabsf(estimated_speed) > 500.0f) {
                // FLYING START: Motor is windmilling fast. Jump to Closed Loop!
                current_state = STATE_CLOSED_LOOP_FOC;
                pll_integ = estimated_speed; // Sync the PLL immediately
                stall_timer = 0;
            } else {
                // Motor has stopped. Go through normal Kick Start.
                current_state = STATE_OFF;
            }
        }
        break;
    }

    case STATE_OFF: {
        //setInverterVoltages(0.0f, 0.0f);
        TIM1->CCER = 0x0000;
        vq_pi_integ = 0.0f;
        vd_pi_integ = 0.0f;

        // DO NOT wipe estimated_speed or smo_i_alpha!
        // Let the observer keep tracking the coasting rotor.
        smo_e_alpha = smo_e_beta = 0.0f;
        pll_integ   = 0.0f;
        // estimated_speed = 0.0f; <--- REMOVED

        if (dshot.throttle <= 0.02f) {
            fault_retry_count = 0;
            fault_retry_timer = 0;
        }

        if (dshot.armed && dshot.throttle > 0.05f) {
            // If the motor is already spinning fast, do a Flying Start!
            if (fabsf(estimated_speed) > 1000.0f) {
                current_state = STATE_CLOSED_LOOP_FOC;
                pll_integ = estimated_speed;
            } else {
                // Otherwise do a normal Kick Start
                tick_counter = 0;
                open_loop_speed = 0.0f;
                open_loop_angle = 0.0f;
                estimated_angle = 0.0f;
                blend_timer = 0;
                current_state = STATE_KICK_START;
            }
        }
        break;
    }

    case STATE_KICK_START: {
        float kick_target = 1000.0f;
        float kick_rate = 2000.0f;

        if (open_loop_speed < kick_target) {
            open_loop_speed += kick_rate * FOC_LOOP_DT;
        } else {
            open_loop_speed = kick_target;
        }

        open_loop_angle += open_loop_speed * FOC_LOOP_DT;
        if (open_loop_angle > _2PI) open_loop_angle -= _2PI;

        float sin_ol, cos_ol;
        cordic_start(open_loop_angle);
        cordic_read(open_loop_angle, &sin_ol, &cos_ol);

        float I_alpha = I_U;
        float I_beta  = -(I_U + 2.0f * I_V) * INV_SQRT3;

        float Id_meas =  I_alpha * cos_ol + I_beta * sin_ol;
        float Iq_meas = -I_alpha * sin_ol + I_beta * cos_ol;

        float Iq_target = 10.0f;
        float Id_target = 0.0f;

        float Iq_err = Iq_target - Iq_meas;
        float Id_err = Id_target - Id_meas;

        const float KP_CURR = 0.1f;
        const float KI_CURR = 200.0f;
        const float V_MAX = v_bus_measured * INV_SQRT3;

        vq_pi_integ += KI_CURR * Iq_err * FOC_LOOP_DT;
        vq_pi_integ = (vq_pi_integ < -V_MAX) ? -V_MAX : ((vq_pi_integ > V_MAX) ? V_MAX : vq_pi_integ);
        float Vq = KP_CURR * Iq_err + vq_pi_integ;

        vd_pi_integ += KI_CURR * Id_err * FOC_LOOP_DT;
        vd_pi_integ = (vd_pi_integ < -V_MAX) ? -V_MAX : ((vd_pi_integ > V_MAX) ? V_MAX : vd_pi_integ);
        float Vd = KP_CURR * Id_err + vd_pi_integ;

        cmd_v_alpha = (Vd * cos_ol) - (Vq * sin_ol);
        cmd_v_beta  = (Vd * sin_ol) + (Vq * cos_ol);

        setInverterVoltages(cmd_v_alpha, cmd_v_beta);

        if (open_loop_speed >= kick_target) {
            // VALIDATION: Did the motor actually follow the open loop?
            // We expect the SMO to see at least SOME speed.
            if (fabsf(estimated_speed) < 300.0f) {
                // The motor didn't spin. It's jammed or desynced.
                current_state = STATE_FAULT;
                fault_retry_timer = 0;
            } else {
                current_state = STATE_FOC_BLEND;
                blend_timer = 0;
                pll_integ = open_loop_speed;
                estimated_speed = open_loop_speed;
                estimated_angle = open_loop_angle;
            }
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
        cordic_start(blended_angle);
        cordic_read(blended_angle, &sin_cl, &cos_cl);

        float I_alpha = I_U;
        float I_beta  = -(I_U + 2.0f * I_V) * INV_SQRT3;

        float Id_meas =  I_alpha * cos_cl + I_beta * sin_cl;
        float Iq_meas = -I_alpha * sin_cl + I_beta * cos_cl;

        float abs_w = (estimated_speed < 0.0f) ? -estimated_speed : estimated_speed;
        float i_limit;
        if (abs_w < 2000.0f) i_limit = 25.0f;
        else if (abs_w > 6000.0f) i_limit = 40.0f;
        else i_limit = 25.0f + 20.0f * (abs_w - 2000.0f) * INV_4000;

        float ramp_Iq_target = 10.0f + ((dshot.throttle * i_limit) - 10.0f) * blend_factor;
        float Iq_target = ramp_Iq_target;
        float Id_target = 0.0f;

        float Iq_err = Iq_target - Iq_meas;
        float Id_err = Id_target - Id_meas;

        const float KP_CURR = 0.1f;
        const float KI_CURR = 200.0f;
        const float V_MAX = v_bus_measured * INV_SQRT3;

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
        cordic_start(advanced_angle);
        cordic_read(advanced_angle, &sin_cl, &cos_cl);

        float I_alpha = I_U;
        float I_beta  = -(I_U + 2.0f * I_V) * INV_SQRT3;

        float Id_meas =  I_alpha * cos_cl + I_beta * sin_cl;
        float Iq_meas = -I_alpha * sin_cl + I_beta * cos_cl;

        float abs_w = (estimated_speed < 0.0f) ? -estimated_speed : estimated_speed;
        float i_limit;
        if (abs_w < 2000.0f) i_limit = 25.0f;
        else if (abs_w > 6000.0f) i_limit = 40.0f;
        else i_limit = 25.0f + 15.0f * (abs_w - 2000.0f) * INV_4000;

        float Iq_target = dshot.throttle * i_limit;
        float Id_target = 0.0f;

        float Iq_err = Iq_target - Iq_meas;
        float Id_err = Id_target - Id_meas;

        const float KP_CURR = 0.1f;
        const float KI_CURR = 200.0f;
        const float V_MAX = v_bus_measured * INV_SQRT3;

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

        // Reset fault retry count if we have been flying healthy
        static uint32_t healthy_timer = 0;
        if (fabsf(Id_meas) < 5.0f && fabsf(Iq_meas) < 20.0f) {
            healthy_timer++;
            if (healthy_timer > 96000) { // 2 seconds at 48kHz
                fault_retry_count = 0;
            }
        } else {
            healthy_timer = 0;
        }

        // --- FAULT DETECTION ---
        // 1. Instant Desync (Id Runaway)
        if (fabsf(Id_meas) > 30.0f) {
            current_state = STATE_FAULT;
            fault_retry_timer = 0;
        }
        // 2. Instant Hard Overcurrent (Raw ADC limit is 41.25A)
        else if (fabsf(I_U) > 39.0f || fabsf(I_V) > 39.0f) {
            current_state = STATE_FAULT;
            fault_retry_timer = 0;
        }
        // 3. Stall Condition (High Iq, Zero Speed)
        else if (fabsf(Iq_meas) > 35.0f && abs_w < 200.0f) {
            stall_timer++;
            if (stall_timer > 500) { // 500 loops at 48kHz = 10.4ms
                current_state = STATE_FAULT;
                fault_retry_timer = 0;
            }
        } else {
            stall_timer = 0; // Reset timer if not stalling
        }

        // Apply 0V immediately if a fault just triggered
        if (current_state == STATE_FAULT) {
            setInverterVoltages(0.0f, 0.0f);
        } else {
            setInverterVoltages(cmd_v_alpha, cmd_v_beta);
        }

        if (dshot.throttle <= 0.02f) {
            current_state = STATE_OFF;
            vq_pi_integ = 0.0f;
            vd_pi_integ = 0.0f;
        }
        break;
    }
    }

    dshot_set_erpm((uint32_t)(estimated_speed * 60.0f * 0.15915494f));
    tick_counter++;

    uint32_t end_cycles = DWT->CYCCNT;
    uint32_t cycles = end_cycles - start_cycles;

    if (cycles > foc_debug.max_cycles) {
        foc_debug.max_cycles = cycles;
    }
    foc_debug.final_state = current_state;
    foc_debug.throttle_x1000 = (uint32_t)(dshot.throttle * 1000.0f);
}

void foc_set_adc_offsets(float u, float v) { offset_U = (int32_t)u; offset_V = (int32_t)v; }
DriveState foc_get_state(void)         { return current_state; }
float foc_get_estimated_speed(void)    { return estimated_speed; }
float foc_get_estimated_angle(void)    { return estimated_angle; }
float foc_get_phase_u_current(void)    { return last_I_U; }
float foc_get_phase_v_current(void)    { return last_I_V; }
float foc_get_v_alpha(void)            { return cmd_v_alpha; }
float foc_get_v_beta(void)             { return cmd_v_beta; }
float foc_get_pll_integ(void)          { return pll_integ; }