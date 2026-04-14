#include "adxl.h"
#include <stdio.h>

// Private Function Prototypes
static void writeRegister(ADXL_HandleTypeDef *dev, uint8_t reg, uint8_t value);
static uint8_t readRegister(ADXL_HandleTypeDef *dev, uint8_t reg);
static HAL_StatusTypeDef readMultipleRegisters(ADXL_HandleTypeDef *dev, uint8_t reg, uint8_t *buffer, uint8_t len);

// Public Functions

bool ADXL_Init(ADXL_HandleTypeDef *dev)
{
  if (dev->SensorType == ADXL345_TYPE)
  {
    dev->g_per_LSB = 0.0040f; // ±16g at 4 mg/LSB
  }
  else if (dev->SensorType == ADXL375_TYPE)
  {
    dev->g_per_LSB = 0.049f; // ±200g at ~49 mg/LSB
  }

  uint8_t deviceId = readRegister(dev, ADXL_REG_DEVID);
  if (deviceId != ADXL_EXPECTED_DEVID)
  {
    printf("Error: Could not find ADXL sensor. Received ID: 0x%02X\r\n", deviceId);
    return false;
  }

  writeRegister(dev, ADXL_REG_POWER_CTL, ADXL_PWR_MEASURE); // Power on the sensor

  if (dev->SensorType == ADXL345_TYPE)
  {
    writeRegister(dev, ADXL_REG_DATA_FORMAT, ADXL_FORMAT_16G); // Resolution and range: ±16g or appropriate setting for ADXL345
  }
  else if (dev->SensorType == ADXL375_TYPE)
  {
    writeRegister(dev, ADXL_REG_DATA_FORMAT, ADXL_FORMAT_16G); // Resolution and range: ±16g or appropriate setting for ADXL375
  }
  return true;
}

void ADXL_ReadAccelerometer(ADXL_HandleTypeDef *dev, float *x, float *y, float *z)
{
  uint8_t buffer[6];

  // Request 6 bytes of data from the ADXL sensor
  // Checking HAL_OK replaces the Wire transmission and byte count checks
  if (readRegisters(dev, ADXL_REG_DATAX0, buffer, 6) != HAL_OK)
  {
    printf("Error: Failed to read from ADXL!\r\n");
    return; // Exit if transmission failed
  }

  // Combine bytes into 16-bit signed values
  int16_t x_raw = (int16_t)(buffer[1] << 8 | buffer[0]);
  int16_t y_raw = (int16_t)(buffer[3] << 8 | buffer[2]);
  int16_t z_raw = (int16_t)(buffer[5] << 8 | buffer[4]);

  // Convert raw values to 'g' units
  *x = x_raw * dev->g_per_LSB;
  *y = y_raw * dev->g_per_LSB;
  *z = z_raw * dev->g_per_LSB;

  // Optional: Print the raw data for debugging
  // printf("X: %f Y: %f Z: %f\r\n", *x, *y, *z);
}

// Private Functions

static void writeRegister(ADXL_HandleTypeDef *dev, uint8_t reg, uint8_t value)
{
  // For SPI writes, we send the register address followed by the value
  uint8_t tx_data[2] = {reg, value};

  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET); // Select sensor
  HAL_SPI_Transmit(dev->hspi, tx_data, 2, HAL_MAX_DELAY);       // Write data
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);   // Deselect sensor
}

static uint8_t readRegister(ADXL_HandleTypeDef *dev, uint8_t reg)
{
  uint8_t address = reg | ADXL_SPI_READ_CMD; // Modify register address for SPI reading
  uint8_t value = 0;

  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET); // Select sensor
  HAL_SPI_Transmit(dev->hspi, &address, 1, HAL_MAX_DELAY);      // Send register to read
  HAL_SPI_Receive(dev->hspi, &value, 1, HAL_MAX_DELAY);         // Receive 1 byte
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);   // Deselect sensor

  return value;
}

static HAL_StatusTypeDef readMultipleRegisters(ADXL_HandleTypeDef *dev, uint8_t reg, uint8_t *buffer, uint8_t len)
{
  uint8_t address = reg | ADXL_SPI_READ_CMD; // Modify register address for SPI reading

  if (len > 1) {
      address |= ADXL_SPI_MB_CMD; // Modify register address for multi-byte reading
  }

  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET); 						// Select sensor
  HAL_SPI_Transmit(dev->hspi, &address, 1, HAL_MAX_DELAY);      						// Send register to read
  HAL_StatusTypeDef status = HAL_SPI_Receive(dev->hspi, buffer, len, HAL_MAX_DELAY); 	// Receive block of data
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);   						// Deselect sensor

  return status; // Return the HAL status to allow error checking in the main function
}


