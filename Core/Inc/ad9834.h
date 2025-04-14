/*
 * ad9834.h
 *
 *  Created on: Mar 10, 2025
 *      Author: matia
 */

#ifndef INC_AD9834_H_
#define INC_AD9834_H_

#include <stdint.h>

//void AD9834_Init(void);
//void AD9834_WriteRegister(uint16_t data);
//void AD9834_SetFrequency(uint8_t reg, uint32_t freq);
//void AD9834_SetPhase(uint8_t reg, uint16_t phase);
//void AD9834_ConfigMCLK(void);

// Function prototypes

void AD9834_Init(void);
void AD9834_WriteRegister(uint16_t data);
void AD9834_SetFrequency(uint8_t reg, uint32_t freq);
void AD9834_SetPhase(uint8_t reg, uint16_t phase);
void AD9834_ConfigMCLK(void);
void AD9834_Setup200kHzSineWave(void);

#endif /* INC_AD9834_H_ */
