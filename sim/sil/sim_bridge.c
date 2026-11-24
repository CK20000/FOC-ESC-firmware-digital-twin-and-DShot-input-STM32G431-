#include "sim_bridge.h"
#include "foc_core.h"
#include "foc_dshot.h"
#include "virtual_stm32.h"
#include <stdlib.h>

SimSession* sim_create(uint32_t seed, float* out_buffer) {
    SimSession* s = (SimSession*)malloc(sizeof(SimSession));
    plant_init(&s->plant, seed);
    s->cycle = 0; s->throttle = 0.0f;
    s->i2_sum_u = 0; s->i2_sum_v = 0; s->i2_sum_w = 0; s->i2_n = 0;
    s->thermal_tick_ready = 0; s->out_buffer = out_buffer;
    foc_set_adc_offsets(2048.0f, 2048.0f);
    foc_setup();
    sil_dshot_set_armed(1);
    return s;
}

void sim_destroy(SimSession* s) { free(s); }

void sim_set_throttle(SimSession* s, float throttle) {
    s->throttle = throttle;
    sil_dshot_set_throttle(throttle);
}

void sim_set_gnd_bounce(SimSession* s, float r_adc, float r_amp) {
    s->plant.R_GND_ADC = r_adc;
    s->plant.R_GND_AMP = r_amp;
}

void sim_reset(SimSession* s, uint32_t seed) {
    plant_init(&s->plant, seed);
    s->cycle = 0; s->i2_sum_u = 0; s->i2_sum_v = 0; s->i2_sum_w = 0; s->i2_n = 0;
    s->thermal_tick_ready = 0;
    foc_set_adc_offsets(2048.0f, 2048.0f);
    foc_setup();
    sil_dshot_set_armed(1);
    sil_dshot_set_throttle(s->throttle);
}

void sim_run(SimSession* s, int num_cycles) {
    for (int i = 0; i < num_cycles; i++) {
        uint32_t ccr1 = sil_tim1_get_ccr1();
        uint32_t ccr2 = sil_tim1_get_ccr2();
        uint32_t ccr3 = sil_tim1_get_ccr3();
        plant_step(&s->plant, ccr1, ccr2, ccr3);
        uint32_t cnt_vbus = plant_get_vbus_counts(&s->plant);
        sil_adc1_set_jdr3(cnt_vbus);

        uint32_t cnt_u, cnt_v;
        plant_get_adc_counts(&s->plant, &cnt_u, &cnt_v);
        sil_adc1_set_jdr1(cnt_u);
        sil_adc2_set_jdr1(cnt_v);
        sil_tim2_advance(3541);
        foc_loop();

        float i_u, i_v;
        plant_get_raw_currents(&s->plant, &i_u, &i_v);
        float i_w = -i_u - i_v;
        s->i2_sum_u += (double)(i_u * i_u);
        s->i2_sum_v += (double)(i_v * i_v);
        s->i2_sum_w += (double)(i_w * i_w);
        s->i2_n++;

        if (s->cycle > 0 && s->cycle % 24000 == 0) {
            s->thermal_i2_avg[0] = (float)(s->i2_sum_u / s->i2_n);
            s->thermal_i2_avg[1] = (float)(s->i2_sum_v / s->i2_n);
            s->thermal_i2_avg[2] = (float)(s->i2_sum_w / s->i2_n);
            s->thermal_tick_ready = 1;
            s->i2_sum_u = 0; s->i2_sum_v = 0; s->i2_sum_w = 0; s->i2_n = 0;
        }

        int idx = i * 15;
        s->out_buffer[idx + 0] = (float)s->cycle;
        s->out_buffer[idx + 1] = (float)ccr1;
        s->out_buffer[idx + 2] = (float)ccr2;
        s->out_buffer[idx + 3] = (float)ccr3;
        s->out_buffer[idx + 4] = s->plant.i_alpha;
        s->out_buffer[idx + 5] = s->plant.i_beta;
        s->out_buffer[idx + 6] = foc_get_phase_u_current();
        s->out_buffer[idx + 7] = foc_get_phase_v_current();
        s->out_buffer[idx + 8] = s->plant.theta;
        s->out_buffer[idx + 9] = foc_get_estimated_angle();
        s->out_buffer[idx + 10] = s->plant.omega;
        s->out_buffer[idx + 11] = foc_get_estimated_speed();
        s->out_buffer[idx + 12] = (float)foc_get_state();
        s->out_buffer[idx + 13] = s->throttle;
        s->out_buffer[idx + 14] = v_bus_measured;

        s->cycle++;
    }
}

int sim_get_thermal_tick(SimSession* s, float* i2_avg) {
    if (s->thermal_tick_ready) {
        i2_avg[0] = s->thermal_i2_avg[0];
        i2_avg[1] = s->thermal_i2_avg[1];
        i2_avg[2] = s->thermal_i2_avg[2];
        s->thermal_tick_ready = 0;
        return 1;
    }
    return 0;
}

void sim_set_stall(SimSession* s, int stall) {
    plant_set_stall(&s->plant, stall);
}