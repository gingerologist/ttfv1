/*
 * tca9555.c
 *
 *  Created on: Mar 11, 2025
 *      Author: matia
 */

/**
 * TCA9555PWR I2C IO Expander Driver
 *
 * This file provides functions to control six TCA9555PWR IO expanders
 * connected to an STM32F405 via I2C1 (PB6/PB7)
 */

#include <stdbool.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tca9555.h"

/* External I2C handle declared in main.c */
extern I2C_HandleTypeDef hi2c1;

/* TCA9555 base address (0x20) shifted left by 1 for I2C operations */
#define TCA9555_BASE_ADDR          (0x20 << 1)

/* Array of device addresses based on A2,A1,A0 pins */
const uint8_t TCA9555_DEVICES[6] =
{
  TCA9555_BASE_ADDR | (0 << 1),  // 000
  TCA9555_BASE_ADDR | (1 << 1),  // 001
  TCA9555_BASE_ADDR | (2 << 1),  // 010
  TCA9555_BASE_ADDR | (3 << 1),  // 011
  TCA9555_BASE_ADDR | (4 << 1),  // 100
  TCA9555_BASE_ADDR | (5 << 1)   // 101
};

/**
 * @brief Write to a register on a specific TCA9555 device
 * @param dev_idx Device index (0-5)
 * @param reg Register address
 * @param value Value to write
 * @return HAL status
 */
HAL_StatusTypeDef TCA9555_WriteReg(uint8_t dev_idx, uint8_t reg, uint8_t value)
{
  uint8_t data[2] =
  { reg,
    value };

  if (dev_idx >= 6)
  {
    return HAL_ERROR;
  }

  return HAL_I2C_Master_Transmit(&hi2c1, TCA9555_DEVICES[dev_idx], data, 2,
      HAL_MAX_DELAY);
}

/**
 * @brief Read from a register on a specific TCA9555 device
 * @param dev_idx Device index (0-5)
 * @param reg Register address
 * @param value Pointer to store the read value
 * @return HAL status
 */
HAL_StatusTypeDef TCA9555_ReadReg(uint8_t dev_idx, uint8_t reg, uint8_t *value)
{
  if (dev_idx >= 6)
  {
    return HAL_ERROR;
  }

  // Send register address
  if (HAL_I2C_Master_Transmit(&hi2c1, TCA9555_DEVICES[dev_idx], &reg, 1,
      HAL_MAX_DELAY) != HAL_OK)
  {
    return HAL_ERROR;
  }

  // Read data
  return HAL_I2C_Master_Receive(&hi2c1, TCA9555_DEVICES[dev_idx], value, 1,
      HAL_MAX_DELAY);
}

HAL_StatusTypeDef TCA9555_ReadAllRegs(uint8_t dev_idx, uint8_t value[8])
{
  HAL_StatusTypeDef status;

  for (uint8_t i = 0; i < 8; i++)
  {
    status = TCA9555_ReadReg(dev_idx, i, &value[i]);
    if (status != HAL_OK)
    {
      return status;
    }
  }

  return HAL_OK;
}

/**
 * @brief Initialize all six TCA9555 IO expanders
 * @return HAL status (HAL_OK if all devices initialized successfully)
 */
HAL_StatusTypeDef TCA9555_Init_All(void)
{
  HAL_StatusTypeDef status = HAL_OK;

  for (uint8_t i = 0; i < 6; i++)
  {
    // Set all outputs low
    if (TCA9555_WriteReg(i, TCA9555_REG_OUTPUT_PORT0, 0x00) != HAL_OK)
    {
      status = HAL_ERROR;
    }

    if (TCA9555_WriteReg(i, TCA9555_REG_OUTPUT_PORT1, 0x00) != HAL_OK)
    {
      status = HAL_ERROR;
    }

    // Set all pins as outputs (0 = output, 1 = input)
    if (TCA9555_WriteReg(i, TCA9555_REG_CONFIG_PORT0, 0x00) != HAL_OK)
    {
      status = HAL_ERROR;
    }

    if (TCA9555_WriteReg(i, TCA9555_REG_CONFIG_PORT1, 0x00) != HAL_OK)
    {
      status = HAL_ERROR;
    }
  }

  return status;
}

/**
 * @brief Write GPIO state for a specific TCA9555 device
 * @param dev_idx Device index (0-5)
 * @param gpio_state 16-bit value for output state
 *                   Lower byte (bits 0-7) = P0.0-P0.7
 *                   Upper byte (bits 8-15) = P1.0-P1.7
 * @return HAL status
 */
HAL_StatusTypeDef TCA9555_WriteGPIO(uint8_t dev_idx, uint16_t gpio_state)
{
  HAL_StatusTypeDef status;

  if (dev_idx >= 6)
  {
    return HAL_ERROR;
  }

  // Write P0 pins (lower byte)
  status = TCA9555_WriteReg(dev_idx, TCA9555_REG_OUTPUT_PORT0,
      (uint8_t) (gpio_state & 0xFF));
  if (status != HAL_OK)
  {
    return status;
  }

  // Write P1 pins (upper byte)
  status = TCA9555_WriteReg(dev_idx, TCA9555_REG_OUTPUT_PORT1,
      (uint8_t) ((gpio_state >> 8) & 0xFF));

  return status;
}

/**
 * @brief Read GPIO state from a specific TCA9555 device
 * @param dev_idx Device index (0-5)
 * @param gpio_state Pointer to store 16-bit value of GPIO state
 *                   Lower byte (bits 0-7) = P0.0-P0.7
 *                   Upper byte (bits 8-15) = P1.0-P1.7
 * @return HAL status
 */
HAL_StatusTypeDef TCA9555_ReadGPIO(uint8_t dev_idx, uint16_t *gpio_state)
{
  HAL_StatusTypeDef status;
  uint8_t port0_value, port1_value;

  if (dev_idx >= 6 || gpio_state == NULL)
  {
    return HAL_ERROR;
  }

  // Read P0 pins (lower byte)
  status = TCA9555_ReadReg(dev_idx, TCA9555_REG_INPUT_PORT0, &port0_value);
  if (status != HAL_OK)
  {
    return status;
  }

  // Read P1 pins (upper byte)
  status = TCA9555_ReadReg(dev_idx, TCA9555_REG_INPUT_PORT1, &port1_value);
  if (status != HAL_OK)
  {
    return status;
  }

  // Combine into 16-bit value
  *gpio_state = ((uint16_t) port1_value << 8) | port0_value;

  return HAL_OK;
}

/**
 * @brief Set a specific pin on a TCA9555 device
 * @param dev_idx Device index (0-5)
 * @param pin Pin number (0-15)
 *            0-7: P0.0-P0.7
 *            8-15: P1.0-P1.7
 * @param state Pin state (0 = low, non-zero = high)
 * @return HAL status
 */
HAL_StatusTypeDef TCA9555_SetPin(uint8_t dev_idx, uint8_t pin, uint8_t state)
{
  HAL_StatusTypeDef status;
  uint16_t current_state;

  if (dev_idx >= 6 || pin >= 16)
  {
    return HAL_ERROR;
  }

  // Read current state
  status = TCA9555_ReadGPIO(dev_idx, &current_state);
  if (status != HAL_OK)
  {
    return status;
  }

  // Modify the specific pin
  if (state)
  {
    current_state |= (1 << pin);  // Set pin high
  }
  else
  {
    current_state &= ~(1 << pin); // Set pin low
  }

  // Write back
  return TCA9555_WriteGPIO(dev_idx, current_state);
}

/**
 * @brief Test function to cycle through all devices sequentially
 * @param delay_ms Delay between each step in milliseconds
 */
void TCA9555_TestSequence(uint32_t delay_ms)
{
  // Pattern to show on each device in sequence
  uint16_t test_pattern = 0x5555;  // Alternating high/low

  // Initialize all devices
  TCA9555_Init_All();

  // Cycle through each device
  while (1)
  {
    for (uint8_t dev = 0; dev < 6; dev++)
    {
      // Turn on test pattern for current device
      TCA9555_WriteGPIO(dev, test_pattern);

      // Wait
      HAL_Delay(delay_ms);

      // Turn off all pins
      TCA9555_WriteGPIO(dev, 0x0000);
    }

    // Invert test pattern for next cycle
    test_pattern = ~test_pattern;
  }
}

const char *bit_rep[16] = {
    [ 0] = "0000", [ 1] = "0001", [ 2] = "0010", [ 3] = "0011",
    [ 4] = "0100", [ 5] = "0101", [ 6] = "0110", [ 7] = "0111",
    [ 8] = "1000", [ 9] = "1001", [10] = "1010", [11] = "1011",
    [12] = "1100", [13] = "1101", [14] = "1110", [15] = "1111",
};

void TCA9555_Pretty(uint8_t dev_idx)
{
  uint8_t value[8];
  HAL_StatusTypeDef status = TCA9555_ReadAllRegs(dev_idx, value);

  if (status != HAL_OK)
  {
    printf("TCA9555 %d: error reading regs\r\n", dev_idx);
    return;
  }

  if (value[6] != 0 || value[7] != 0)
  {
    printf("TCA9555 %d: some pins are input, CFG0: 0x%02x, CFG1: 0x%02x\r\n",
        dev_idx, value[6], value[7]);
    return;
  }

  if (value[4] != 0 || value[5] != 0)
  {
    printf("TCA9555 %d: unexpected polarity, POL0: 0x%02x, POL1: 0x%02x\r\n",
        dev_idx, value[4], value[5]);
    return;
  }

  printf("TCA9555 %d: output0: %s%s, output1: %s%s\r\n", dev_idx,
      bit_rep[value[2] >> 4], bit_rep[value[2] & 0x0F], bit_rep[value[3] >> 4],
      bit_rep[value[3] & 0x0F]);
}


void TCA9555_Dump(void)
{
  for (int i = 0; i < 6; i++)  // For each I/O expander
  {
    TCA9555_Pretty(i);
  }
}

/**
 *
 */
void TCA9555_UpdateOutput(uint8_t port[6][2])
{
  for (int i = 0; i < 6; i++)
  {
    TCA9555_WriteReg(i, TCA9555_REG_OUTPUT_PORT0, 0);
    TCA9555_WriteReg(i, TCA9555_REG_OUTPUT_PORT1, 0);
  }

  vTaskDelay(50);   // FreeRTOS.h, task.h
  // osDelay(50);   // cmsis_os.h

  for (int i = 0; i < 6; i++)
  {
    TCA9555_WriteReg(i, TCA9555_REG_OUTPUT_PORT0, port[i][0]);
    TCA9555_WriteReg(i, TCA9555_REG_OUTPUT_PORT1, port[i][1]);
  }
}
