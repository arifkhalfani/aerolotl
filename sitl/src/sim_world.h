/*
 * sim_world.h - owns ONE shared instance of ground truth for the whole
 * simulated flight. Both accel_sim.c and baro_sim.c read from this same
 * instance, so they can never disagree about what altitude the rocket
 * is "really" at.
 *
 * This file only exists in the SITL build. There is no equivalent for
 * real firmware - reality plays that role instead.
 */

#ifndef SIM_WORLD_H
#define SIM_WORLD_H

void  sim_world_init(void);
void  sim_world_step(float dt); /* advance ground truth by dt seconds */

float sim_world_true_altitude(void);
float sim_world_true_velocity(void);
float sim_world_proper_accel(void); /* what an ideal, noise-free accelerometer would read */

#endif /* SIM_WORLD_H */