/**
 * @file    Adxl.h
 * @brief   STM32 HAL SPI driver for ADXL345 (±16g) and ADXL375 (±200g)
 *          accelerometers.
 *
 * Port of the Arduino I2C driver to STM32 HAL SPI (mode 3).
 *
 * Both parts share an identical register map and command set; they differ only
 * in measurement range and per-LSB scaling. A single driver handles both via
 * the ADXL_Type enum.
 *
 * --- CubeMX SPI configuration ---
 *   Mode:            Full-Duplex Master
 *   Data Size:       8 bits
 *   First Bit:       MSB First
 *   CPOL:            High         (Clock Polarity)
 *   CPHA:            2 Edge       (Clock Phase)   --> SPI Mode 3
 *   NSS:             Software (CS managed in this driver)
 *   Baud rate:       <= 5 MHz (set prescaler accordingly)
 *
 * --- CubeMX CS pin configuration ---
 *   GPIO Output, Push-Pull, No Pull-up/down, High Speed, initial level HIGH.
 *   Pass the resulting GPIO_Port + GPIO_Pin to ADXL_Init().
 *
 * --- Usage ---
 *   ADXL_HandleTypeDef adxl345;
 *   ADXL_Init(&adxl345, &hspi1, ADXL345_CS_GPIO_Port, ADXL345_CS_Pin,
 *             ADXL_TYPE_345);
 *   if (!ADXL_Begin(&adxl345)) { error_handler(); }
 *
 *   float x, y, z;
 *   if (ADXL_ReadAccelerometer(&adxl345, &x, &y, &z)) {
 *       // x, y, z are in units of g
 *   }
 */

#ifndef ADXL_H
#define ADXL_H

#include "main.h"      /* CubeMX-generated; provides HAL types */
#include <stdbool.h>
#include <stdint.h>

/** Selects which device the handle represents. Sets the per-LSB scaling. */
typedef enum {
    ADXL_TYPE_345 = 0,   /* ±16g full resolution, 4 mg/LSB */
    ADXL_TYPE_375 = 1    /* ±200g, ~49 mg/LSB */
} ADXL_Type;

/** Driver instance. Initialize with ADXL_Init() before any other call. */
typedef struct {
    SPI_HandleTypeDef *hspi;       /* SPI peripheral handle (e.g. &hspi1) */
    GPIO_TypeDef      *cs_port;    /* CS port (e.g. GPIOA) */
    uint16_t           cs_pin;     /* CS pin mask (e.g. GPIO_PIN_4) */
    ADXL_Type          type;       /* ADXL345 or ADXL375 */
    float              g_per_LSB;  /* Set by ADXL_Init based on type */
} ADXL_HandleTypeDef;

/**
 * @brief  Populate the handle. Does not touch hardware.
 * @note   The CS line is not driven here. Configure it as GPIO output HIGH
 *         in CubeMX so it's de-asserted before ADXL_Begin() runs.
 */
void ADXL_Init(ADXL_HandleTypeDef *dev,
               SPI_HandleTypeDef  *hspi,
               GPIO_TypeDef       *cs_port,
               uint16_t            cs_pin,
               ADXL_Type           type);

/**
 * @brief  Probe the device and configure measurement mode.
 * @return true if WHO_AM_I matches and config writes succeed; false otherwise.
 */
bool ADXL_Begin(ADXL_HandleTypeDef *dev);

/**
 * @brief  Read the X/Y/Z acceleration registers and convert to g.
 * @return true on success, false on SPI error.
 */
bool ADXL_ReadAccelerometer(ADXL_HandleTypeDef *dev,
                            float *x, float *y, float *z);

#endif /* ADXL_H */
