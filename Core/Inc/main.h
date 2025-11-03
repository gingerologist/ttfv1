/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint32_t DDS_FreqReg(double f_out);
void DDS_Start(uint32_t freq, bool dry_run);

void DAC_Start(void);
void DAC_SetOutput_Percent(uint32_t percentage);

void PA4_GPIO_High(void);
void PA4_Restore_DAC(void);

void DAC_Disable(void);
void DAC_Enable(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW1_Pin GPIO_PIN_0
#define SW1_GPIO_Port GPIOC
#define SW2_Pin GPIO_PIN_1
#define SW2_GPIO_Port GPIOC
#define SW3_Pin GPIO_PIN_2
#define SW3_GPIO_Port GPIOC
#define SW4_Pin GPIO_PIN_3
#define SW4_GPIO_Port GPIOC
#define MUX1_CLR_Pin GPIO_PIN_0
#define MUX1_CLR_GPIO_Port GPIOB
#define MUX1_LE_N_Pin GPIO_PIN_1
#define MUX1_LE_N_GPIO_Port GPIOB
#define DDS_RESET_Pin GPIO_PIN_6
#define DDS_RESET_GPIO_Port GPIOC
#define DDS_FSEL_Pin GPIO_PIN_9
#define DDS_FSEL_GPIO_Port GPIOA
#define DDS_PSEL_Pin GPIO_PIN_10
#define DDS_PSEL_GPIO_Port GPIOA
#define LEDC_Pin GPIO_PIN_11
#define LEDC_GPIO_Port GPIOA
#define SW5_Pin GPIO_PIN_12
#define SW5_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
