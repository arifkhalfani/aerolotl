/*
 * physics_sim.h / physics_sim.c
 *
 * Minimal 1D vertical ballistic model: motor burn (constant proper
 * acceleration) then coast (proper acceleration = 0, gravity only).
 * NO DRAG - this is a deliberate simplification, not an oversight.
 * Real SITL (per WP-05) needs Cd_body/Cd_brake and air density vs
 * altitude; this is the smallest model that produces a distinguishable
 * boost phase vs coast phase, which is all we need to see the filter
 * behave differently in each regime.
 *
 * Ground truth only - the filter never sees anything in this file
 * directly. main.c reads sim_state_t.true_h/true_v purely to PRINT a
 * comparison, exactly like a real SITL harness compares estimate vs
 * simulator truth.
 */

#ifndef PHYSICS_SIM_H
#define PHYSICS_SIM_H

#define SIM_G 9.80665f

typedef struct {
    float t;            /* mission time, s */
    float true_h;        /* true altitude, m */
    float true_v;         /* true vertical velocity, m/s */
    float proper_accel;    /* what an accelerometer would read, m/s^2 */
} sim_state_t;

/* Motor burn duration and thrust-phase proper acceleration - tune these
   to taste. Kept small so a demo run reaches apogee in well under a
   minute of wall-clock time instead of needing to sit through a
   realistic multi-minute coast with no drag to slow it down. */
#define SIM_BURN_TIME_S      2.0f
#define SIM_THRUST_ACCEL     100.0f /* proper accel during burn, m/s^2 */

void sim_init(sim_state_t *s);
void sim_step(sim_state_t *s, float dt); /* advances true_h/true_v/proper_accel by dt */

/* Simple Gaussian-ish noise generator (Box-Muller), mean 0, given std-dev.
   Fine for a demo; not cryptographically anything, don't reuse elsewhere. */
float sim_gaussian_noise(float sigma);

#endif /* PHYSICS_SIM_H */