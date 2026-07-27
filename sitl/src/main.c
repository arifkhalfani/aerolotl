/*
 * Aerolotl SITL skeleton - step 3: real (simulated) sensor fusion.
 *
 * SensorReadTask: steps the physics sim forward, derives noisy fake
 *   sensor readings from ground truth (proper acceleration + barometric
 *   pressure), pushes them onto a queue.
 * StateEstimationTask: consumes readings, runs the Kalman filter
 *   (predict on accel, update on pressure), prints estimate vs truth.
 *
 * Ground truth (sim_state_t.true_h/true_v) is used ONLY for the
 * comparison printout - the filter never touches it directly. This
 * mirrors how a real SITL harness grades the estimator against the
 * simulator's truth, not something the flight code itself can see.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "physics_sim.h"
#include "kalman.h"

#define BARO_P0_PA        101325.0f
#define BARO_SCALE_H_M    8434.5f
#define BARO_NOISE_PA     3.0f    /* ~BMP581-ish RMS noise, Pa */
#define ACCEL_NOISE_MPS2  0.05f   /* accelerometer noise, m/s^2 */

typedef struct {
    uint32_t seq;
    float    dt;                 /* seconds since last sample, for the predict step */
    float    accel_measured;     /* noisy proper acceleration, m/s^2 */
    float    pressure_measured;  /* noisy barometric pressure, Pa */
    float    true_h;             /* ground truth - NOT fed to the filter, print-only */
    float    true_v;             /* ground truth - NOT fed to the filter, print-only */
} sensor_sample_t;

static QueueHandle_t xSensorQueue;

/* --- Task 1: physics sim + fake sensors --- */
static void SensorReadTask(void *pvParameters)
{
    (void) pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const float dt = 0.02f; /* 50 Hz */

    sim_state_t sim;
    sim_init(&sim);

    uint32_t seq = 0;

    for (;;) {
        sim_step(&sim, dt);

        float true_pressure = BARO_P0_PA * expf(-sim.true_h / BARO_SCALE_H_M);

        sensor_sample_t sample = {
            .seq = seq++,
            .dt = dt,
            .accel_measured = sim.proper_accel + sim_gaussian_noise(ACCEL_NOISE_MPS2),
            .pressure_measured = true_pressure + sim_gaussian_noise(BARO_NOISE_PA),
            .true_h = sim.true_h,
            .true_v = sim.true_v,
        };

        if (xQueueSend(xSensorQueue, &sample, 0) != pdPASS) {
            printf("[SensorRead] queue full, dropped sample %u\n", sample.seq);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}

/* --- Task 2: Kalman filter consuming the queue --- */
static void StateEstimationTask(void *pvParameters)
{
    (void) pvParameters;
    sensor_sample_t sample;

    kalman_state_t k;
    kalman_init(&k, 0.0f, 0.0f, 1.0f, 1.0f);

    for (;;) {
        if (xQueueReceive(xSensorQueue, &sample, portMAX_DELAY) == pdPASS) {

            /* IMPORTANT: accelerometer reads PROPER acceleration, not
               kinematic. Subtract gravity before using it as the
               filter's control input - see chat discussion. */
            float kinematic_accel_estimate = sample.accel_measured - SIM_G;

            kalman_predict(&k, kinematic_accel_estimate, sample.dt, ACCEL_NOISE_MPS2);
            kalman_update_baro_pressure(&k, sample.pressure_measured, BARO_NOISE_PA,
                                         BARO_P0_PA, BARO_SCALE_H_M);

            if (sample.seq % 25 == 0) { /* print twice a second */
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
    printf("Aerolotl SITL - Kalman filter tracking simulated boost/coast flight\n");
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