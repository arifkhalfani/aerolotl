/*
 * Aerolotl SITL - step 4: sensors.h seam in place.
 *
 * SensorReadTask now talks ONLY to sensors.h (accel_read/baro_read).
 * It has zero knowledge that a simulator exists underneath - swap
 * accel_sim.c/baro_sim.c for accel_stm32.c/baro_stm32.c in the Makefile
 * and this task's code does not change.
 *
 * The one exception, clearly marked below: sim_world_step() has to be
 * called by SOMETHING once per loop to advance ground truth. That call
 * lives here because this main.c is itself the SITL harness - it will
 * never be the main.c used on real firmware (CubeIDE has its own). A
 * future common/flight_tasks.c (shared between sim and firmware) would
 * NOT contain this call; it would live only in harness code like this.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "sensors.h"
#include "sim_world.h"     /* SITL-harness only - see note above */
#include "physics_sim.h"   /* only for SIM_G, SIM_BURN_TIME_S, SIM_THRUST_ACCEL (banner + gravity const) */
#include "kalman.h"

#define BARO_P0_PA        101325.0f /* estimator's assumed atmosphere model -
                                        happens to equal baro_sim.c's constant
                                        today, but represents a different thing:
                                        what WE assume, not what's "true" */
#define BARO_SCALE_H_M    8434.5f
#define BARO_NOISE_PA     3.0f     /* estimator's assumed sensor noise (R) -
                                        would come from a datasheet on real hw */
#define ACCEL_NOISE_MPS2  0.05f    /* estimator's assumed process noise (Q input) */

typedef struct {
    uint32_t seq;
    float    dt;
    float    accel_measured;     /* from sensors.h, not sim internals */
    float    pressure_measured;  /* from sensors.h, not sim internals */
    float    true_h;             /* ground truth - NOT fed to the filter, print-only */
    float    true_v;             /* ground truth - NOT fed to the filter, print-only */
} sensor_sample_t;

static QueueHandle_t xSensorQueue;

/* --- Task 1: reads sensors.h only. No sim types appear below this line. --- */
static void SensorReadTask(void *pvParameters)
{
    (void) pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const float dt = 0.02f; /* 50 Hz */

    accel_init();
    baro_init();
    sim_world_init(); /* SITL-harness only */

    uint32_t seq = 0;

    for (;;) {
        sim_world_step(dt); /* SITL-harness only - advances ground truth */

        accel_sample_t accel;
        baro_sample_t baro;
        accel_read(&accel); /* -> accel_sim.c today, accel_stm32.c later */
        baro_read(&baro);   /* -> baro_sim.c today, baro_stm32.c later */

        sensor_sample_t sample = {
            .seq = seq++,
            .dt = dt,
            .accel_measured = accel.proper_accel_mps2,
            .pressure_measured = baro.pressure_pa,
            .true_h = sim_world_true_altitude(), /* SITL-harness only, print-use */
            .true_v = sim_world_true_velocity(), /* SITL-harness only, print-use */
        };

        if (xQueueSend(xSensorQueue, &sample, 0) != pdPASS) {
            printf("[SensorRead] queue full, dropped sample %u\n", sample.seq);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}

/* --- Task 2: unchanged from before - never touched the sim directly anyway --- */
static void StateEstimationTask(void *pvParameters)
{
    (void) pvParameters;
    sensor_sample_t sample;

    kalman_state_t k;
    kalman_init(&k, 0.0f, 0.0f, 1.0f, 1.0f);

    for (;;) {
        if (xQueueReceive(xSensorQueue, &sample, portMAX_DELAY) == pdPASS) {

            float kinematic_accel_estimate = sample.accel_measured - SIM_G;

            kalman_predict(&k, kinematic_accel_estimate, sample.dt, ACCEL_NOISE_MPS2);
            kalman_update_baro_pressure(&k, sample.pressure_measured, BARO_NOISE_PA,
                                         BARO_P0_PA, BARO_SCALE_H_M);

            if (sample.seq % 25 == 0) {
                printf("[t=%5.2fs] est: h=%8.2f m  v=%7.2f m/s   |   true: h=%8.2f m  v=%7.2f m/s   |   err: h=%+6.2f m\n",
                       sample.seq * sample.dt,
                       k.h, k.v,
                       sample.true_h, sample.true_v,
                       k.h - sample.true_h);
            }
        }
    }
}

int main(void)
{
    setvbuf(stdout, NULL, _IOLBF, 0);
    printf("Aerolotl SITL - sensors.h seam in place (sim-backed today)\n");
    printf("(burn=%.1fs, thrust_accel=%.1f m/s^2 - see physics_sim.h to tune)\n\n",
           SIM_BURN_TIME_S, SIM_THRUST_ACCEL);

    xSensorQueue = xQueueCreate(10, sizeof(sensor_sample_t));
    if (xSensorQueue == NULL) {
        printf("Failed to create queue\n");
        return 1;
    }

    xTaskCreate(SensorReadTask, "SensorRead", configMINIMAL_STACK_SIZE, NULL,
                tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(StateEstimationTask, "StateEst", configMINIMAL_STACK_SIZE, NULL,
                tskIDLE_PRIORITY + 2, NULL);

    vTaskStartScheduler();

    printf("Scheduler exited unexpectedly\n");
    return 1;
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void) xTask;
    printf("STACK OVERFLOW in task: %s\n", pcTaskName);
    exit(1);
}

void vApplicationMallocFailedHook(void)
{
    printf("MALLOC FAILED\n");
    exit(1);
}