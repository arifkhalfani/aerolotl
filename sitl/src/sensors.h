/*
 * sensors.h - the hardware abstraction seam.
 *
 * This is THE file that makes the flight software portable across
 * "run on my laptop against fake sensors" and "run on real silicon".
 * Rules for this file, permanently:
 *   - No HAL types (no SPI_HandleTypeDef, no GPIO pin numbers).
 *   - No sim types (no sim_state_t, no physics constants).
 *   - Describes what a sensor PROVIDES, never how it's obtained.
 *
 * Two backends implement this today:
 *   accel_sim.c / baro_sim.c   - read from sim_world (SITL)
 * Future backend (unwritten):
 *   accel_stm32.c / baro_stm32.c - wraps your real Adxl.c / BMP581 driver
 *
 * Only one backend per sensor gets compiled in at a time - selected by
 * which .c file the Makefile lists, not by anything in this header.
 */

#ifndef SENSORS_H
#define SENSORS_H

typedef struct {
    float proper_accel_mps2; /* specific force along the sensitive axis - NOT
                                 kinematic acceleration, gravity not included */
} accel_sample_t;

typedef struct {
    float pressure_pa;
} baro_sample_t;

/* Returns 0 on success, negative on failure. Matches the shape a real
   HAL wrapper would return (map HAL_OK -> 0, HAL_ERROR -> negative)
   without this header knowing HAL_StatusTypeDef exists. */
int accel_init(void);
int accel_read(accel_sample_t *out);

int baro_init(void);
int baro_read(baro_sample_t *out);

#endif /* SENSORS_H */