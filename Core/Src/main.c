/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <sys/stat.h>
#include <errno.h>
#include <math.h>
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "profile.h"
#include "ad9834.h"
// #include "tca9555.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

DAC_HandleTypeDef hdac;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

osThreadId profileTaskHandle;
osThreadId uxTaskHandle;
osMessageQId requestQueueHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI2_Init(void);
static void MX_DAC_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
void StartProfileTask(void const *argument);
void StartUxTask(void const *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  Overrides the standard _write function to redirect stdout and stderr
 * to UART2
 * @param  file: File descriptor (1 for stdout, 2 for stderr)
 * @param  ptr: Pointer to the data buffer to be written
 * @param  len: Number of bytes to write
 * @retval The number of bytes written if successful, -1 otherwise
 *
 * @note   This function is called by printf and other stdio functions to output
 * data. It redirects standard output (stdout) and standard error (stderr) to
 * the configured UART2 peripheral, enabling printf functionality over serial.
 */
int _write(int file, char *ptr, int len) {
  // Redirect stdout (file=1) and stderr (file=2) to UART2
  if (file == 1 || file == 2) {
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
  }

  errno = EBADF;
  return -1;
}

/*
---- test freq reg ----
200KHz, hi reg 16bit is 0x4083
200KHz, lo reg 16bit is 0x449b

 */

//  uint64_t test = DDS_FreqReg(200000);
//  printf("\r\n\r\n---- test freq reg ----\r\n");
//  printf("200KHz -> uint64_t, high 32bit is 0x%08x\r\n", (uint32_t)(test >>
//  32)); printf("200KHz -> uint64_t,  low 32bit is 0x%08x\r\n",
//  (uint32_t)test);
uint32_t DDS_FreqReg(double f_out) {
  uint32_t f32 = (uint32_t)round(f_out * 10.73741824);
  uint32_t lo = (f32 & 0x00003fff) | 0x00004000;
  uint32_t hi = (((f32 << 2) & 0xffff0000)) | 0x40000000;

  return hi | lo;
}

/**
 * @brief  Initializes and starts the AD9834 DDS (Direct Digital Synthesis) chip
 * @retval None
 *
 * https://www.analog.com/en/resources/app-notes/an-1070.html
 *
 * @note   This function configures and enables the AD9834 DDS chip from Analog
 * Devices by sending the following sequence via SPI:
 *         1. Control word with B28=1 and RESET=1 (reset chip, enable 28-bit
 * frequency writes)
 *         2. Frequency register low word (FREQLW)
 *         3. Frequency register high word (FREQHW)
 *         4. Phase register configuration
 *         5. Control word with B28=1 and RESET=0 (maintain config, enable
 * output)
 */
void DDS_Start(uint32_t freq, bool dry_run) {
  if (freq < 10000) {
    freq = 10000;
  }

  uint32_t freq_reg = DDS_FreqReg((double)freq);
  uint16_t freq_lo = (uint16_t)(freq_reg & 0x0000FFFF);
  uint16_t freq_hi = ((uint16_t)((freq_reg >> 16) & 0x0000FFFF));

  uint16_t data[5] = {0x2100, // Control word: B28=1, RESET=1
                      freq_lo,
                      // 0x449C, // FREQLW for 200KHz
                      freq_hi,
                      // 0x4083,  // FREQHW for 200KHz
                      0xC000,  // Phase register
                      0x2000}; // Control word: B28=1, RESET=0 (enable output)

  printf("freq: %lu, lo: 0x%04x, hi: 0x%04x\r\n", freq, freq_lo, freq_hi);

  for (int i = 0; i < 5; i++) {
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&data[i], 1, HAL_MAX_DELAY);
  }
}

/**
 * @brief  Starts the DAC channel 1 conversion
 * @retval None
 *
 * @note   This is a simple wrapper function that calls the HAL DAC start
 * function for DAC channel 1 using the pre-configured DAC handle (hdac)
 */
void DAC_Start(void) { HAL_DAC_Start(&hdac, DAC_CHANNEL_1); }

/**
 * @brief  Updates the DAC output voltage to the specified value in millivolts
 * @param  mv: Desired output voltage in millivolts (mV)
 * @retval None
 *
 * @note   This function converts the requested voltage from millivolts to the
 *         corresponding 12-bit DAC value (0-4095) based on a 3.3V reference
 * voltage, then updates DAC channel 1 with the calculated value
 */
// void DAC_Update(uint32_t mv)
//{
//   uint32_t value = (mv * 4095) / 3300; /* Convert 700mV to DAC value */
//   HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value);
// }

/**
 * @brief  Sets DAC OUT1 based on percentage input
 * @param  percentage: Input value from 0 to 100
 *         0%   -> 1150mV output
 *         100% -> 0V output
 * @retval None
 */
void DAC_SetOutput_Percent(uint32_t percentage) {
  //  static uint32_t last_percentage = 0;
  //
  //  // Validate input range
  //  if (percentage > 100)
  //  {
  //    percentage = 100;
  //  }
  //
  //  if (percentage == last_percentage)
  //  {
  //    return;
  //  }

  // Calculate DAC value
  // STM32F405 has 12-bit DAC (0-4095)
  // VREF = 3.3V
  // For 0% -> 1150mV (value = 1150/3300*4095)
  // For 100% -> 0V (value = 0)
  // Linear interpolation for values in between

  // 1150mV corresponds to DAC value of 1426 (1150/3300*4095)
  // 1150mV 1426
  // 1180mV 1464 <- nominal value
  // 1200mV 1489
  // 1210mV 1502 <- (nominal + max) / 2
  // 1240mV 1539
  // 1250mV 1551
  uint32_t value = 1502 - (percentage * 1502 / 100);

  // Write value to DAC channel 1
  // DAC_SetChannel1Data(DAC_Align_12b_R, dac_value);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value);

  // printf("DAC out percentage: %lu, value: %lu\r\n", percentage, value);

  // Start DAC Channel1
  // DAC_Cmd(DAC_Channel_1, ENABLE);
}

void PA4_GPIO_High(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);
  HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);

  // Method 2: Set output register BEFORE HAL_GPIO_Init (avoids glitch)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,
                    GPIO_PIN_SET); // Pre-load output register
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void PA4_Restore_DAC(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

  // Reconfigure PA4 back to analog mode for DAC
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // The DAC should now control the pin again
  // Optionally write a new value to ensure proper output:
  // HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, your_dac_value);
}

void DAC_Disable(void) {
  // Stop: Clear the EN bit
  hdac.Instance->CR &= ~DAC_CR_EN1; // For channel 1
}

void DAC_Enable(void) {
  // Restart: Set the EN bit
  hdac.Instance->CR |= DAC_CR_EN1; // For channel 1
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_SPI2_Init();
  MX_DAC_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of requestQueue */
  osMessageQDef(requestQueue, 1, profile_t);
  requestQueueHandle = osMessageCreate(osMessageQ(requestQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of profileTask */
  osThreadDef(profileTask, StartProfileTask, osPriorityAboveNormal, 0, 2048);
  profileTaskHandle = osThreadCreate(osThread(profileTask), NULL);

  /* definition and creation of uxTask */
  osThreadDef(uxTask, StartUxTask, osPriorityNormal, 0, 2048);
  uxTaskHandle = osThreadCreate(osThread(uxTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data
   * Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */
}

/**
 * @brief DAC Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC_Init(void) {

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
   */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK) {
    Error_Handler();
  }

  /** DAC channel OUT1 config
   */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  __HAL_SPI_ENABLE(&hspi1);
  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MUX1_LE_N_Pin | MUX1_CLR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LEDC_GPIO_Port, LEDC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SW1_Pin SW2_Pin SW3_Pin SW4_Pin */
  GPIO_InitStruct.Pin = SW1_Pin | SW2_Pin | SW3_Pin | SW4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX1_LE_N_Pin */
  GPIO_InitStruct.Pin = MUX1_LE_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MUX1_LE_N_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX1_CLR_Pin */
  GPIO_InitStruct.Pin = MUX1_CLR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MUX1_CLR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DDS_RESET_Pin */
  GPIO_InitStruct.Pin = DDS_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DDS_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DDS_FSEL_Pin DDS_PSEL_Pin */
  GPIO_InitStruct.Pin = DDS_FSEL_Pin | DDS_PSEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LEDC_Pin */
  GPIO_InitStruct.Pin = LEDC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEDC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW5_Pin */
  GPIO_InitStruct.Pin = SW5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW5_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartProfileTask */
/**
 * @brief  Function implementing the profileTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartProfileTask */
__weak void StartProfileTask(void const *argument) {
  /* USER CODE BEGIN 5 */
  //  /* Infinite loop */
  //  // GPIO_PinState ps = GPIO_PIN_SET;
  //  // const uint8_t digipot = 192;
  //  // const uint8_t hello[] = "hello\r\n";
  //  // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  //
  //  uart_printf(&huart2, "\r\n--- ttf boot ---\r\n");
  //  // AD9834_ConfigMCLK();
  //  // AD9834_Setup200kHzSineWave();
  //  // +AD9834_ConfigMCLK();
  //  TestAD9834();
  //
  //  print_tca9555();
  //  TCA9555_Init_All();
  //  print_tca9555();
  //
  //  // set P36 - P40 C (com), and P41-P45 S (signal)
  //  // chip 4 P0, chip 5 P0 and P1 involved
  //  // chip 4 P0 -> 00010101 (P36-38C)
  //  // chip 5 P1 -> 10100101 (P39-40C, P41-42S)
  //  // chip 5 P0 -> 10101000 (P43-45S, two lsbs irrelevent)
  //  // TCA9555_WriteReg(4, TCA9555_REG_OUTPUT_PORT0, 0b00010111);
  //
  //  TCA9555_WriteReg(4, TCA9555_REG_OUTPUT_PORT0, 0b00010101);
  //  TCA9555_WriteReg(5, TCA9555_REG_OUTPUT_PORT1, 0b10100101);
  //  TCA9555_WriteReg(5, TCA9555_REG_OUTPUT_PORT0, 0b10101000);
  //
  //
  //  for(int i = 0;;i++)
  //  {
  //
  //    // HAL_GPIO_WritePin();
  //    // HAL_GPIO_TogglePin(ledc_GPIO_Port, ledc_Pin);
  //
  //    // HAL_GPIO_WritePin(ledc_GPIO_Port, ledc_Pin, ps);
  //
  //    // HAL_UART_Transmit(&huart2, hello, 7, HAL_MAX_DELAY);
  //    // HAL_SPI_Transmit(&hspi1, &digipot, 1, HAL_MAX_DELAY);
  //
  //
  //    uart_printf(&huart2, "## %d\r\n", i);
  //
  ////    for (int j = 0; j < 6; j++)
  ////    {
  ////      TCA9555_WriteReg(j, TCA9555_REG_OUTPUT_PORT0, 0x00);
  ////      TCA9555_WriteReg(j, TCA9555_REG_OUTPUT_PORT1, 0x00);
  ////    }
  //
  ////    TCA9555_WriteReg(5, TCA9555_REG_OUTPUT_PORT0, 0x00);
  ////    if (i % 2 == 0)
  ////    {
  ////      regval = 1 << 2;
  ////    }
  ////    else
  ////    {
  ////      regval = 1 << 3;
  ////    }
  ////
  ////    TCA9555_WriteReg(5, TCA9555_REG_OUTPUT_PORT0, regval);
  //
  //
  //
  //
  //    // os neutral version of vTaskDelay(1000);
  //    osDelay(5000);
  //  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartUxTask */
/**
 * @brief Function implementing the uxTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUxTask */
__weak void StartUxTask(void const *argument) {
  /* USER CODE BEGIN StartUxTask */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END StartUxTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM2 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
