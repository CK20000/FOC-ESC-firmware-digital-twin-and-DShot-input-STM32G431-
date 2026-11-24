#include "plant_c.h"
#include <math.h>
#include <stdlib.h>

#define PI 3.14159265359f
#define TWO_PI 6.28318530718f

static inline uint32_t xorshift32(uint32_t *state) {
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

static inline float rand_gauss(PlantState* p, float mean, float std) {
    float sum = 0.0f;
    for (int i = 0; i < 4; i++) {
        sum += (float)(xorshift32(&p->rng_state) & 0xFFFF) / 65535.0f;
    }
    return mean + (sum - 2.0f) * std * 1.732f;
}

static inline float rand_uniform(PlantState* p, float min, float max) {
    float r = (float)(xorshift32(&p->rng_state) & 0xFFFF) / 65535.0f;
    return min + r * (max - min);
}

static inline float soft_sign(float x) {
    return x / sqrtf(x*x + 0.1f);
}

void plant_init(PlantState* p, uint32_t seed) {
    p->RS = 0.110f; p->LS = 30e-6f; p->LAMBDA = 0.00041f;
    p->POLE_PAIRS = 7.0f; p->INERTIA = 5e-6f; p->V_BUS_OC = 25.2f;
    p->DT_SUB = 5e-6f; p->BETA_SIGN = -1.0f; p->DAMPING = 2e-5f;
    p->PROP_LOAD = 1.2e-7f; p->R_BATT = 0.020f; p->R_DS_ON = 0.00335f;
    p->T_DEAD = 200e-9f; p->F_PWM = 48000.0f; p->T_PWM = 1.0f / p->F_PWM;
    p->AMPS_PER_COUNT = (3.3f / 4095.0f) / (0.002f * 20.0f);
    p->MAX_READABLE_AMPS = 41.25f; p->SAMPLE_JITTER = 0.05f;

    p->R_GND_ADC = 0.0f;
    p->R_GND_AMP = 0.0f;
    p->stall = 0;

    p->rng_state = seed ? seed : 12345;

    p->off_u = rand_gauss(p, 0, 15);
    p->off_v = rand_gauss(p, 0, 15);
    p->gain_u = 1.0f + rand_gauss(p, 0, 0.01f);
    p->gain_v = 1.0f + rand_gauss(p, 0, 0.01f);

    p->theta = rand_uniform(p, 0, TWO_PI);
    p->i_alpha = 0; p->i_beta = 0; p->omega = 0;
    p->v_bus_sag = p->V_BUS_OC;
    p->i_u_filt = 0; p->i_v_filt = 0;

    p->ccr_pipe[0][0] = 885; p->ccr_pipe[0][1] = 885; p->ccr_pipe[0][2] = 885;
    p->ccr_pipe[1][0] = 885; p->ccr_pipe[1][1] = 885; p->ccr_pipe[1][2] = 885;
}

void plant_get_raw_currents(PlantState* p, float* i_u, float* i_v) {
    float i_a = p->i_alpha;
    float i_b_eff = p->i_beta * p->BETA_SIGN;
    *i_u = i_a;
    *i_v = -0.5f * i_a + 0.8660254f * i_b_eff;
}

void plant_step(PlantState* p, uint32_t ccr1, uint32_t ccr2, uint32_t ccr3) {
    p->ccr_pipe[0][0] = p->ccr_pipe[1][0]; p->ccr_pipe[0][1] = p->ccr_pipe[1][1]; p->ccr_pipe[0][2] = p->ccr_pipe[1][2];
    p->ccr_pipe[1][0] = ccr1; p->ccr_pipe[1][1] = ccr2; p->ccr_pipe[1][2] = ccr3;

    uint32_t d_ccr1 = p->ccr_pipe[0][0];
    uint32_t d_ccr2 = p->ccr_pipe[0][1];
    uint32_t d_ccr3 = p->ccr_pipe[0][2];

    float i_mag = sqrtf(p->i_alpha*p->i_alpha + p->i_beta*p->i_beta);
    p->v_bus_sag = p->V_BUS_OC - (i_mag * 0.707f * p->R_BATT);

    float da = (float)d_ccr1 / 1770.0f;
    float db = (float)d_ccr2 / 1770.0f;
    float dc = (float)d_ccr3 / 1770.0f;

    float va = (da - 0.5f) * p->v_bus_sag;
    float vb = (db - 0.5f) * p->v_bus_sag;
    float vc = (dc - 0.5f) * p->v_bus_sag;

    float i_u, i_v;
    plant_get_raw_currents(p, &i_u, &i_v);
    float i_w = -i_u - i_v;

    float v_dead_scale = (p->T_DEAD / p->T_PWM) * p->v_bus_sag;
    va -= v_dead_scale * soft_sign(i_u);
    vb -= v_dead_scale * soft_sign(i_v);
    vc -= v_dead_scale * soft_sign(i_w);

    va -= i_u * 2.0f * p->R_DS_ON;
    vb -= i_v * 2.0f * p->R_DS_ON;
    vc -= i_w * 2.0f * p->R_DS_ON;

    float vcom = (va + vb + vc) / 3.0f;
    va -= vcom; vb -= vcom; vc -= vcom;

    float v_alpha = va;
    float v_beta = (va + 2.0f * vb) / 1.7320508f;

    int n_sub = 4;
    float dt = 20.833e-6f;
    float dts = dt / n_sub;
    float load_torque = rand_uniform(p, -0.002f, 0.002f);

    for (int i = 0; i < n_sub; i++) {
        float e_alpha = -p->LAMBDA * p->omega * sinf(p->theta);
        float e_beta = p->LAMBDA * p->omega * cosf(p->theta);

        float di_alpha = (v_alpha - p->RS * p->i_alpha - e_alpha) / p->LS;
        float di_beta = (v_beta - p->RS * p->i_beta - e_beta) / p->LS;

        p->i_alpha += di_alpha * dts;
        p->i_beta += di_beta * dts;

        float i_q = -p->i_alpha * sinf(p->theta) + p->i_beta * cosf(p->theta);
        float torque = 1.5f * p->POLE_PAIRS * p->LAMBDA * i_q;

        float omega_mech = p->omega / p->POLE_PAIRS;
        float prop_torque = p->PROP_LOAD * omega_mech * fabsf(omega_mech);
        float alpha_mech = (torque - load_torque - prop_torque - p->DAMPING * omega_mech) / p->INERTIA;

         omega_mech += alpha_mech * dts;

        if (p->stall) {
            omega_mech = 0.0f; // Force rotor to stop immediately
        }

        p->omega = omega_mech * p->POLE_PAIRS;
        p->theta += p->omega * dts;

        while (p->theta >= TWO_PI) p->theta -= TWO_PI;
        while (p->theta < 0.0f) p->theta += TWO_PI;
    }
}

void plant_get_adc_counts(PlantState* p, uint32_t* cnt_u, uint32_t* cnt_v) {
    float i_u, i_v;
    plant_get_raw_currents(p, &i_u, &i_v);
    float i_w = -i_u - i_v;

    float i_total_gnd = fabsf(i_u) + fabsf(i_v) + fabsf(i_w);

    // Differential ground bounce
    float v_gnd_amp = i_total_gnd * p->R_GND_AMP;
    float v_gnd_adc = i_total_gnd * p->R_GND_ADC;
    float v_diff_error = v_gnd_amp - v_gnd_adc;

    float alpha_filt = 0.9812f;
    p->i_u_filt += alpha_filt * (i_u - p->i_u_filt);
    p->i_v_filt += alpha_filt * (i_v - p->i_v_filt);

    float u_clip = fmaxf(-p->MAX_READABLE_AMPS, fminf(p->MAX_READABLE_AMPS, p->i_u_filt));
    float v_clip = fmaxf(-p->MAX_READABLE_AMPS, fminf(p->MAX_READABLE_AMPS, p->i_v_filt));

    float cnt_u_f = 2048.0f + p->off_u + p->gain_u * (u_clip / p->AMPS_PER_COUNT);
    float cnt_v_f = 2048.0f + p->off_v + p->gain_v * (v_clip / p->AMPS_PER_COUNT);

    // Apply differential error
    float v_error_counts = v_diff_error / (0.002f * 20.0f) / p->AMPS_PER_COUNT;
    cnt_u_f += v_error_counts;
    cnt_v_f += v_error_counts;

    float d[3] = { (float)p->ccr_pipe[0][0]/1770.0f, (float)p->ccr_pipe[0][1]/1770.0f, (float)p->ccr_pipe[0][2]/1770.0f };
    float max_d = fmaxf(d[0], fmaxf(d[1], d[2]));
    float min_d = fminf(d[0], fminf(d[1], d[2]));
    float spread = max_d - min_d;
    float ripple_pp = (p->v_bus_sag * spread * (1.0f - spread)) / (p->LS * p->F_PWM);

    float jitter_u = ripple_pp * p->SAMPLE_JITTER * rand_uniform(p, -1, 1) / p->AMPS_PER_COUNT;
    float jitter_v = ripple_pp * p->SAMPLE_JITTER * rand_uniform(p, -1, 1) / p->AMPS_PER_COUNT;

    float emi_scale = 0.05f * fabsf(p->i_u_filt) + 0.05f * fabsf(p->i_v_filt);
    float emi_u = emi_scale * rand_uniform(p, -1, 1);
    float emi_v = emi_scale * rand_uniform(p, -1, 1);

    int32_t final_u = (int32_t)roundf(cnt_u_f + jitter_u + emi_u);
    int32_t final_v = (int32_t)roundf(cnt_v_f + jitter_v + emi_v);

    *cnt_u = (uint32_t)fmaxf(0, fminf(4095, final_u));
    *cnt_v = (uint32_t)fmaxf(0, fminf(4095, final_v));
}

void plant_set_stall(PlantState* p, int stall) {
    p->stall = stall;
}

uint32_t plant_get_vbus_counts(PlantState* p) {
    // V_bus -> Divider (1/11) -> ADC (0-4095)
    float v_pin = p->v_bus_sag / 11.0f;
    float counts = (v_pin / 3.3f) * 4095.0f;
    return (uint32_t)fmaxf(0, fminf(4095, counts));
}