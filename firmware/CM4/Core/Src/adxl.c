/**
 * @file    Adxl.c
 * @brief   STM32 HAL SPI driver for ADXL345 / ADXL375. See Adxl.h for usage.
 */

#include "Adxl.h"

/* --- Register map (same for ADXL345 and ADXL375) --- */
#define ADXL_REG_DEVID        0x00  /* Device ID, fixed 0xE5 */
#define ADXL_REG_POWER_CTL    0x2D  /* Power control */
#define ADXL_REG_DATA_FORMAT  0x31  /* Data format */
#define ADXL_REG_DATAX0       0x32  /* X-axis data low byte (start of 6 bytes) */

#define ADXL_DEVID_EXPECTED   0xE5

/* --- SPI command-byte bits ---
 * Byte 1 of every SPI transaction:
 *   bit7 = R/W  (1 = read,  0 = write)
 *   bit6 = MB   (1 = multi-byte, 0 = single-byte)
 *   bit5..0 = register address
 */
#define ADXL_SPI_READ         0x80
#define ADXL_SPI_MULTIBYTE    0x40

/* SPI timeout (ms). Transactions are tiny so this is generous. */
#define ADXL_SPI_TIMEOUT_MS   100

/* --- Internal helpers --- */

static inline void adxl_cs_low(ADXL_HandleTypeDef *dev) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void adxl_cs_high(ADXL_HandleTypeDef *dev) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

/**
 * Write a single byte to a register.
 * Tx frame: [reg & 0x3F][value]
 */
static bool adxl_write_register(ADXL_HandleTypeDef *dev,
                                uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    tx[0] = reg & 0x3F;     /* R/W=0 (write), MB=0 (single) */
    tx[1] = value;

    adxl_cs_low(dev);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(dev->hspi, tx, 2,
                                                ADXL_SPI_TIMEOUT_MS);
    adxl_cs_high(dev);
    return (status == HAL_OK);
}

/**
 * Read a single register.
 * Tx frame: [reg | 0x80][dummy]
 * Rx frame: [garbage][value]
 */
static bool adxl_read_register(ADXL_HandleTypeDef *dev,
                               uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = { (uint8_t)(reg | ADXL_SPI_READ), 0x00 };
    uint8_t rx[2] = { 0 };

    adxl_cs_low(dev);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 2,
                                                       ADXL_SPI_TIMEOUT_MS);
    adxl_cs_high(dev);

    if (status != HAL_OK) {
        return false;
    }
    *value = rx[1];
    return true;
}

/**
 * Read len consecutive registers starting at reg.
 * Tx frame: [reg | 0xC0][dummy x len]
 * Rx frame: [garbage][data x len]
 *
 * len must be <= 31 (the ADXL register space cap; in practice we read 6).
 */
static bool adxl_read_registers(ADXL_HandleTypeDef *dev,
                                uint8_t reg, uint8_t *buffer, uint8_t len)
{
    /* Stack buffers sized for the largest read we ever do (6-byte data block).
     * If you extend this driver to do larger reads, bump these. */
    uint8_t tx[7] = { 0 };
    uint8_t rx[7] = { 0 };

    if (len == 0 || len > 6) {
        return false;
    }

    tx[0] = reg | ADXL_SPI_READ | ADXL_SPI_MULTIBYTE;

    adxl_cs_low(dev);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(dev->hspi, tx, rx,
                                                       len + 1,
                                                       ADXL_SPI_TIMEOUT_MS);
    adxl_cs_high(dev);

    if (status != HAL_OK) {
        return false;
    }
    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = rx[i + 1];
    }
    return true;
}

/* --- Public API --- */

void ADXL_Init(ADXL_HandleTypeDef *dev,
               SPI_HandleTypeDef  *hspi,
               GPIO_TypeDef       *cs_port,
               uint16_t            cs_pin,
               ADXL_Type           type)
{
    dev->hspi    = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin  = cs_pin;
    dev->type    = type;

    /* Match the scaling used by the Arduino driver. */
    if (type == ADXL_TYPE_345) {
        dev->g_per_LSB = 0.0040f;   /* ±16g, full-res mode: ~3.9 mg/LSB */
    } else { /* ADXL_TYPE_375 */
        dev->g_per_LSB = 0.049f;    /* ±200g: ~49 mg/LSB */
    }
}

bool ADXL_Begin(ADXL_HandleTypeDef *dev)
{
    /* WHO_AM_I check. Both ADXL345 and ADXL375 return 0xE5. */
    uint8_t devid = 0;
    if (!adxl_read_register(dev, ADXL_REG_DEVID, &devid)) {
        return false;
    }
    if (devid != ADXL_DEVID_EXPECTED) {
        return false;
    }

    /* POWER_CTL = 0x08 -> exit standby, enter measure mode. */
    if (!adxl_write_register(dev, ADXL_REG_POWER_CTL, 0x08)) {
        return false;
    }

    /* DATA_FORMAT = 0x0B
     *   bit 3 (FULL_RES) = 1  -> full resolution mode (4 mg/LSB on ADXL345;
     *                            ignored on ADXL375 which is fixed 13-bit)
     *   bits 1:0 (RANGE) = 11 -> ±16g (ignored on ADXL375 which is fixed ±200g)
     *   bit 6 (SPI) = 0       -> 4-wire SPI (default)
     * Matches the Arduino driver's behavior on both parts.
     */
    if (!adxl_write_register(dev, ADXL_REG_DATA_FORMAT, 0x0B)) {
        return false;
    }

    return true;
}

bool ADXL_ReadAccelerometer(ADXL_HandleTypeDef *dev,
                            float *x, float *y, float *z)
{
    uint8_t buf[6];
    if (!adxl_read_registers(dev, ADXL_REG_DATAX0, buf, 6)) {
        return false;
    }

    /* Each axis is little-endian 16-bit signed across two registers. */
    int16_t x_raw = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    int16_t y_raw = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
    int16_t z_raw = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);

    *x = (float)x_raw * dev->g_per_LSB;
    *y = (float)y_raw * dev->g_per_LSB;
    *z = (float)z_raw * dev->g_per_LSB;

    return true;
}
