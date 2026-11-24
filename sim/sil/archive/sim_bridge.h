#ifndef SIM_BRIDGE_H
#define SIM_BRIDGE_H

#include <stdint.h>
#include "plant_c.h"

typedef struct {
    PlantState plant;
    uint32_t cycle;
    float throttle;

    // Thermal Accumulators
    double i2_sum_u, i2_sum_v, i2_sum_w;
    uint32_t i2_n;
    int thermal_tick_ready;
    float thermal_i2_avg[3];

    // Output buffer (size 14 per cycle)
    float* out_buffer;
} SimSession;

SimSession* sim_create(uint32_t seed, float* out_buffer);
void sim_destroy(SimSession* s);
void sim_set_throttle(SimSession* s, float throttle);
void sim_run(SimSession* s, int num_cycles);
int sim_get_thermal_tick(SimSession* s, float* i2_avg);
void sim_reset(SimSession* s, uint32_t seed);
void sim_set_gnd_bounce(SimSession* s, float r_adc, float r_amp);

#endif