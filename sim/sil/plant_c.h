#ifndef PLANT_C_H
#define PLANT_C_H

#include <stdint.h>

typedef struct {
    // Parameters
    float RS, LS, LAMBDA, POLE_PAIRS, INERTIA, V_BUS_OC, DT_SUB;
    float BETA_SIGN, DAMPING, PROP_LOAD;
    float R_BATT, R_DS_ON, T_DEAD, F_PWM, T_PWM;
    float AMPS_PER_COUNT, MAX_READABLE_AMPS, SAMPLE_JITTER;

    // Spatial Ground Bounce
    float R_GND_ADC;  // MCU side ground resistance
    float R_GND_AMP;  // Op-Amp side ground resistance
    int stall;

    // Sensor Calibration
    float off_u, off_v, gain_u, gain_v;

    // State Variables
    float theta, i_alpha, i_beta, omega, v_bus_sag;
    float i_u_filt, i_v_filt;

    // PWM Delay Pipeline
    uint32_t ccr_pipe[2][3];

    // RNG state
    uint32_t rng_state;
} PlantState;

void plant_init(PlantState* p, uint32_t seed);
void plant_step(PlantState* p, uint32_t ccr1, uint32_t ccr2, uint32_t ccr3);
void plant_get_raw_currents(PlantState* p, float* i_u, float* i_v);
void plant_get_adc_counts(PlantState* p, uint32_t* cnt_u, uint32_t* cnt_v);
void plant_set_stall(PlantState* p, int stall);
uint32_t plant_get_vbus_counts(PlantState* p);

#endif