/*
 * Aerolotl SITL skeleton: confirm FreeRTOS POSIX port runs.
 *
 * Two tasks in one queue, deliberately shaped like the real architecture:
 *   SensorReadTask      -> pushes a fake reading onto a queue at a fixed rate
 *   StateEstimationTask -> blocks on the queue, "processes" the reading
 *
 * Nothing here talks to a sensor yet. The point of this file is to prove
 * the toolchain (kernel + POSIX port + your config) actually builds and
 * runs two independent, rate-differentiated tasks communicating through
 * a queue - the core mechanic the whole flight software depends on.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

typedef struct {
    uint32_t seq;
    float    fake_pressure_pa;
} sensor_sample_t;

static QueueHandle_t xSensorQueue;

/* --- Task 1: stands in for your real SensorRead task --- */
static void SensorReadTask(void *pvParameters)
{
    (void) pvParameters;
    uint32_t seq = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        sensor_sample_t sample = {
            .seq = seq++,
            .fake_pressure_pa = 101325.0f - (float) seq * 0.5f /* pretend we're climbing */
        };

        if (xQueueSend(xSensorQueue, &sample, 0) != pdPASS) {
            printf("[SensorRead] queue full, dropped sample %u\n", sample.seq);
        }

        /* 50 Hz, matches your airbrake-task rate as a placeholder */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}

/* --- Task 2: stands in for your real StateEstimation task --- */
static void StateEstimationTask(void *pvParameters)
{
    (void) pvParameters;
    sensor_sample_t sample;

    for (;;) {
        if (xQueueReceive(xSensorQueue, &sample, portMAX_DELAY) == pdPASS) {
            if (sample.seq % 50 == 0) { /* print once a second so output is readable */
                printf("[StateEstimation] seq=%u pressure=%.2f Pa\n",
                       sample.seq, sample.fake_pressure_pa);
            }
        }
    }
}

int main(void)
{
    setvbuf(stdout, NULL, _IOLBF, 0); /* line-buffer stdout even when piped */
    printf("Aerolotl SITL skeleton - FreeRTOS POSIX port\n");

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

    /* vTaskStartScheduler() only returns if it ran out of heap for the
       idle/timer tasks - if you see this print, configTOTAL_HEAP_SIZE
       is too small. */
    printf("Scheduler exited unexpectedly\n");
    return 1;
}

/* --- Required hooks given configCHECK_FOR_STACK_OVERFLOW / malloc-failed --- */
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