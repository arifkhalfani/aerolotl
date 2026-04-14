#ifndef ADXL_H
#define ADXL_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define ADXL_REG_DEVID       0x00
#define ADXL_REG_POWER_CTL   0x2D
#define ADXL_REG_DATA_FORMAT 0x31
#define ADXL_REG_DATAX0      0x32

#define ADXL_EXPECTED_DEVID  0xE5
#define ADXL_PWR_MEASURE     0x08
#define ADXL_FORMAT_16G      0x0B // Full resolution, ±16g

// SPI specific bitmasks (required by ADXL datasheet for SPI communication)
#define ADXL_SPI_READ_CMD    0x80 // Bit 7 must be 1 for a read operation
#define ADXL_SPI_MB_CMD      0x40 // Bit 6 must be 1 for multi-byte read operations

typedef enum {
    ADXL345_TYPE,
    ADXL375_TYPE
} ADXL_TYPE_t;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    ADXL_TYPE_t SensorType;
    float g_per_LSB;
} ADXL_HandleTypeDef;


// Public Functions
bool ADXL_Init(ADXL_HandleTypeDef *dev);
void ADXL_ReadAccelerometer(ADXL_HandleTypeDef *dev, float *x, float *y, float *z);

#endif
