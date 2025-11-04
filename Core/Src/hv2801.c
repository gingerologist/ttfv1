/*
 * hv2801.c
 *
 *  Created on: Nov 4, 2025
 *      Author: matia
 */

/* HV2801 High Voltage Switch Controller for STM32F405
 * SPI1: PB3 (SCK), PB5 (MOSI)
 * Mode 3: CPOL=1, CPHA=2Edge
 * Software NSS control via GPIO
 */

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include "main.h"
#include "hv2801.h"

#define USE_SPI

#ifdef USE_SPI
// External SPI handle (defined in your main or spi configuration file)
extern SPI_HandleTypeDef hspi1;
#endif

/**
 * @brief Initialize HV2801 control pins
 * @note Call this once at startup after GPIO clocks are enabled
 */
void HV2801_Init(void) {
  // Set LE_N = HIGH (idle state, not latching)
  HAL_GPIO_WritePin(MUX1_LE_N_GPIO_Port, MUX1_LE_N_Pin, GPIO_PIN_SET);
  // Set CLR = LOW permanently (normal operation mode)
  HAL_GPIO_WritePin(MUX1_CLR_GPIO_Port, MUX1_CLR_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Write 32-bit configuration to HV2801
 * @param config: 32-bit switch configuration
 *                Bit 0-1: SW0 (00=OFF, 01=OFF, 10=OFF, 11=ON)
 *                Bit 2-3: SW1
 *                ...
 *                Bit 30-31: SW15
 * @note For your application:
 *       - Odd switches (SW1,3,5...31): Connected to AC signal
 *       - Even switches (SW0,2,4...30): Connected to GND
 *       - Use 0b11 to turn ON a switch, 0b00 for OFF
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef HV2801_WriteConfig(uint32_t config) {
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t data[4];

  // Convert 32-bit word to byte array (MSB first for shift register)
  // HV2801 shifts in MSB first (D31 first, D0 last)
  data[0] = (config >> 24) & 0xFF; // Bits 31-24 (SW15-SW12)
  data[1] = (config >> 16) & 0xFF; // Bits 23-16 (SW11-SW8)
  data[2] = (config >> 8) & 0xFF;  // Bits 15-8  (SW7-SW4)
  data[3] = (config >> 0) & 0xFF;  // Bits 7-0   (SW3-SW0)

  // Start transaction: LE_N = LOW (enable data flow)
  HAL_GPIO_WritePin(MUX1_LE_N_GPIO_Port, MUX1_LE_N_Pin, GPIO_PIN_RESET);

  // Small delay for LE setup time (adjust if needed, typically ~10ns)
  // May not be needed at typical MCU speeds, but included for safety
  __NOP();
  __NOP();
  __NOP();

#ifdef USE_SPI

  // Transmit all 32 bits (4 bytes)
  status = HAL_SPI_Transmit(&hspi1, data, 4, HAL_MAX_DELAY);

  // Ensure SPI transmission is complete
  while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
    ;
#endif

  // Small delay for data hold time
  __NOP();
  __NOP();
  __NOP();

  // End transaction: LE_N = HIGH (rising edge latches all 32 bits)
  HAL_GPIO_WritePin(MUX1_LE_N_GPIO_Port, MUX1_LE_N_Pin, GPIO_PIN_SET);

  return status;
}

/**
 * @brief Helper function to build switch configuration
 * @param switch_num: Switch number (0-15)
 * @param state: 0 = OFF, non-zero = ON
 * @param current_config: Current 32-bit configuration
 * @retval Updated 32-bit configuration
 */
uint32_t HV2801_SetSwitch(uint8_t switch_num, uint8_t state,
                          uint32_t current_config) {
  if (switch_num > 15)
    return current_config;

  uint8_t bit_pos = switch_num * 2;

  // Clear both bits for this switch
  current_config &= ~(0x03 << bit_pos);

  // Set bits if ON (11b = ON)
  if (state) {
    current_config |= (0x03 << bit_pos);
  }

  return current_config;
}

/**
 * @brief Emergency shutoff - turn all switches OFF immediately
 * @note Uses CLR signal for instant shutoff without SPI transaction
 */
void HV2801_EmergencyOff(void) {
  // CLR = HIGH forces all switches OFF
  HAL_GPIO_WritePin(MUX1_CLR_GPIO_Port, MUX1_CLR_Pin, GPIO_PIN_SET);
}

void HV2801_CLR(void) {
  HAL_GPIO_WritePin(MUX1_CLR_GPIO_Port, MUX1_CLR_Pin, GPIO_PIN_RESET);
  vTaskDelay(1);
  HAL_GPIO_WritePin(MUX1_CLR_GPIO_Port, MUX1_CLR_Pin, GPIO_PIN_SET);
  vTaskDelay(1);
  HAL_GPIO_WritePin(MUX1_CLR_GPIO_Port, MUX1_CLR_Pin, GPIO_PIN_RESET);
  vTaskDelay(1);
}

void HV2801_SW(int index) {

  if (index < 0 || index > 31) {
    printf("ERROR: HV2801_SW: invalid index %d (0..31)\r\n", index);
    return;
  }

  uint32_t config = ((uint32_t)1) << index;

  HAL_GPIO_WritePin(MUX1_CLR_GPIO_Port, MUX1_CLR_Pin, GPIO_PIN_RESET);
  vTaskDelay(1);
  HAL_GPIO_WritePin(MUX1_CLR_GPIO_Port, MUX1_CLR_Pin, GPIO_PIN_SET);
  vTaskDelay(1);
  HAL_GPIO_WritePin(MUX1_CLR_GPIO_Port, MUX1_CLR_Pin, GPIO_PIN_RESET);
  vTaskDelay(1);

  HAL_StatusTypeDef status = HV2801_WriteConfig(config);
  if (status != HAL_OK) {
    printf("FAIL: HV2801: status %d\r\n", status);
  }
}

/**
 * @brief Re-enable normal operation after emergency shutoff
 */
void HV2801_Enable(void) {
  // CLR = LOW allows normal operation
  HAL_GPIO_WritePin(MUX1_CLR_GPIO_Port, MUX1_CLR_Pin, GPIO_PIN_RESET);
}

/* ===== EXAMPLE USAGE ===== */
#if 0
void Example_Usage(void)
{
    uint32_t config = 0;

    // Initialize HV2801
    HV2801_Init();

    // Example 1: Turn ON switch 1 (odd - connected to AC signal)
    config = HV2801_SetSwitch(1, 1, config);
    HV2801_WriteConfig(config);

    // Example 2: Turn ON switches 1, 3, 5 (multiple odd switches)
    config = 0;
    config = HV2801_SetSwitch(1, 1, config);
    config = HV2801_SetSwitch(3, 1, config);
    config = HV2801_SetSwitch(5, 1, config);
    HV2801_WriteConfig(config);

    // Example 3: Direct 32-bit write (turn ON only SW1)
    // SW1 uses bits 2-3, so 0b11 at position 2 = 0x0000000C
    HV2801_WriteConfig(0x0000000C);

    // Example 4: Emergency shutoff
    HV2801_EmergencyOff();
    HAL_Delay(100);
    HV2801_Enable();  // Re-enable
}
#endif
