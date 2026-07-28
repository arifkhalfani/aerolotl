#include "sim_world.h"
#include "physics_sim.h"

static sim_state_t g_world;

void sim_world_init(void)
{
    sim_init(&g_world);
}

void sim_world_step(float dt)
{
    sim_step(&g_world, dt);
}

float sim_world_true_altitude(void) { return g_world.true_h; }
float sim_world_true_velocity(void) { return g_world.true_v; }
float sim_world_proper_accel(void)  { return g_world.proper_accel; }