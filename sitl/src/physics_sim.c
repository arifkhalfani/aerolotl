#include <math.h>
#include <stdlib.h>
#include "physics_sim.h"

void sim_init(sim_state_t *s)
{
    s->t = 0.0f;
    s->true_h = 0.0f;
    s->true_v = 0.0f;
    s->proper_accel = SIM_THRUST_ACCEL;
}

void sim_step(sim_state_t *s, float dt)
{
    /* Proper acceleration: what the accelerometer reads. Thrust/mass
       during burn, ~0 during coast (no drag modeled). */
    s->proper_accel = (s->t < SIM_BURN_TIME_S) ? SIM_THRUST_ACCEL : 0.0f;

    /* Kinematic acceleration: what actually moves the rocket.
       proper_accel is specific force; gravity is NOT sensed by the
       accelerometer but still acts on the vehicle. */
    float kinematic_accel = s->proper_accel - SIM_G;

    /* Simple semi-implicit Euler integration - fine for a demo at 50Hz;
       a real SITL would want RK4 for accuracy over a full flight. */
    s->true_v += kinematic_accel * dt;
    s->true_h += s->true_v * dt;

    if (s->true_h < 0.0f) { /* crude "landed" clamp, ignores parachute entirely */
        s->true_h = 0.0f;
        s->true_v = 0.0f;
    }

    s->t += dt;
}

float sim_gaussian_noise(float sigma)
{
    /* Box-Muller transform: two uniforms -> one Gaussian sample. */
    float u1 = ((float) rand() + 1.0f) / ((float) RAND_MAX + 1.0f); /* avoid log(0) */
    float u2 = (float) rand() / (float) RAND_MAX;
    float z = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float) M_PI * u2);
    return z * sigma;
}