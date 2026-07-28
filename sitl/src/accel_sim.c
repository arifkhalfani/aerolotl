/*
 * accel_sim.c - implements sensors.h's accel_read() using sim_world.
 * This is the ONLY file that would be swapped for accel_stm32.c later.
 */

#include "sensors.h"
#include "sim_world.h"
#include "physics_sim.h" /* for sim_gaussian_noise() */

/* Noise injected into the FAKE reading - represents "how noisy is the
   physical sensor we're pretending exists". A real accel_stm32.c has
   no equivalent of this; the noise there is whatever the real chip
   actually produces. */
#define ACCEL_SIM_NOISE_MPS2 0.05f

int accel_init(void)
{
    return 0; /* nothing to configure for a fake sensor */
}

int accel_read(accel_sample_t *out)
{
    out->proper_accel_mps2 = sim_world_proper_accel()
                              + sim_gaussian_noise(ACCEL_SIM_NOISE_MPS2);
    return 0;
}