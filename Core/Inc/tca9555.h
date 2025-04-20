/*
 * tca9555.h
 *
 *  Created on: Mar 11, 2025
 *      Author: matia
 */

#ifndef INC_TCA9555_H_
#define INC_TCA9555_H_

#include "stm32f4xx_hal.h"

/* TCA9555 Register addresses */
#define TCA9555_REG_INPUT_PORT0    0x00  // Input port 0
#define TCA9555_REG_INPUT_PORT1    0x01  // Input port 1
#define TCA9555_REG_OUTPUT_PORT0   0x02  // Output port 0
#define TCA9555_REG_OUTPUT_PORT1   0x03  // Output port 1
#define TCA9555_REG_POLARITY_PORT0 0x04  // Polarity inversion port 0
#define TCA9555_REG_POLARITY_PORT1 0x05  // Polarity inversion port 1
#define TCA9555_REG_CONFIG_PORT0   0x06  // Configuration port 0
#define TCA9555_REG_CONFIG_PORT1   0x07  // Configuration port 1

HAL_StatusTypeDef TCA9555_WriteReg(uint8_t dev_idx, uint8_t reg, uint8_t value);
HAL_StatusTypeDef TCA9555_ReadReg(uint8_t dev_idx, uint8_t reg, uint8_t *value);
HAL_StatusTypeDef TCA9555_Init_All(void);

void TCA9555_Dump();
void TCA9555_UpdateOutput(uint8_t port[6][2]);

#endif /* INC_TCA9555_H_ */
