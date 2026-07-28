/*
 * baro_sim.c - implements sensors.h's baro_read() using sim_world.
 * This is the ONLY file that would be swapped for baro_stm32.c later.
 *
 * Note: this file converts the sim's true altitude into a pressure
 * reading (matching what a real barometer would actually output - raw
 * pressure, not altitude). That conversion uses the same barometric
 * formula the ESTIMATOR uses to go the other way - see the note in
 * main.c about why these constants are duplicated rather than shared.
 */

#include <math.h>
#include "sensors.h"
#include "sim_world.h"
#include "physics_sim.h" /* for sim_gaussian_noise() */

#define BARO_SIM_P0_PA       101325.0f
#define BARO_SIM_SCALE_H_M   8434.5f
#define BARO_SIM_NOISE_PA    3.0f /* ~BMP581-ish RMS noise */

int baro_init(void)
{
    return 0;
}

int baro_read(baro_sample_t *out)
{
    float true_pressure = BARO_SIM_P0_PA
                           * expf(-sim_world_true_altitude() / BARO_SIM_SCALE_H_M);
    out->pressure_pa = true_pressure + sim_gaussian_noise(BARO_SIM_NOISE_PA);
    return 0;
}