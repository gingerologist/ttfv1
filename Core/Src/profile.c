/*
 * control.c
 *
 *  Created on: Apr 23, 2024
 *      Author: ma
 */
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "profile.h"

#define STOP_PROFILE profile[STOP_PROFILE_INDEX]
#define DDBF_PROFILE profile[DDBF_PROFILE_INDEX]
#define CURR_PROFILE profile[CURR_PROFILE_INDEX]
#define NEXT_PROFILE profile[NEXT_PROFILE_INDEX]

extern osMessageQId requestQueueHandle;

static profile_v2_t profile[NUM_OF_ALL_PROFILES] = {
    {}, // 0
    {}, // 1
    {}, // 2
    {}, // 3
    {}, // 4
    {}, // 5
    {}, // 6
    {}, // 7
    {}, // 8
    {}, // 9
    {}, // 10
    {}, // 11
    {}, // 12
    {}, // 13
    {}, // 14
    {}, // 15
    {}, // 16, stop
    {}, // 17, deadbeef is kicking again
    {}, // 18, current
    {}, // 19, next
};

extern CRC_HandleTypeDef hcrc;
extern ADC_HandleTypeDef hadc1;

static HAL_StatusTypeDef save_profiles(void);
static HAL_StatusTypeDef load_profiles(void);

static uint16_t adc_dma_buf[960];

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* Public user code ---------------------------------------------------------*/

static char cfg2char(unsigned int cfg) {
  if (cfg == 0) {
    return '0';
  } else if (cfg == 1) {
    return '1';
  } else if (cfg == 2) {
    return '2';
  } else {
    return '*';
  }
}

static void print_allpads_str_v2(allpads_v2_t *pads, char str[20]) {
  str[0] = cfg2char(pads->a1);
  str[1] = cfg2char(pads->a2);
  str[2] = cfg2char(pads->a3);
  str[3] = cfg2char(pads->a4);
  str[4] = ',';
  str[5] = cfg2char(pads->b1);
  str[6] = cfg2char(pads->b2);
  str[7] = cfg2char(pads->b3);
  str[8] = cfg2char(pads->b4);
  str[9] = ',';
  str[10] = cfg2char(pads->c1);
  str[11] = cfg2char(pads->c2);
  str[12] = cfg2char(pads->c3);
  str[13] = cfg2char(pads->c4);
  str[14] = ',';
  str[15] = cfg2char(pads->d1);
  str[16] = cfg2char(pads->d2);
  str[17] = cfg2char(pads->d3);
  str[18] = cfg2char(pads->d4);
  str[19] = '\0'; // null termination
}

void print_profile(int i) {
  static char str[54];

  if (i >= NUM_OF_ALL_PROFILES) {
    return;
  }

  print_allpads_str_v2(&profile[i].a.pads, str);
  printf("Profile #%02d phase a: %s in %d seconds at %d volts and %lu Hz\r\n",
         i, str, profile[i].a.duration, profile[i].a.level, profile[i].a.freq);

  print_allpads_str_v2(&profile[i].b.pads, str);
  printf("            phase b: %s in %d seconds at %d volts and %lu Hz\r\n",
         str, profile[i].b.duration, profile[i].b.level, profile[i].b.freq);
}

profile_v2_t get_profile(int index) {
  profile_v2_t prfl = {0};

  if (index > -1 && index < 16) {
    return profile[index];
  }
  return prfl;
}

void set_profile_phase(int profile_index, int phase_index,
                       const phase_v2_t *phase) {
  if (profile_index < 0 || profile_index > LAST_PROFILE_INDEX) {
    return;
  }

  if (phase_index == 0) {
    profile[profile_index].a = *phase;
  } else {
    profile[profile_index].b = *phase;
  }

  if (profile_index < NUM_OF_PROFILES) {
    save_profiles();
  }
}

typedef struct {
  uint8_t addr;    // 0-6
  uint8_t port;    // 0 or 1
  uint8_t mask[3]; // see netmap initializer
} netmap_t;

extern const char *bit_rep[16];

void do_profile(int index) {
  if (index < 0 || index > LAST_PROFILE_INDEX) {
    printf("error: do_profile, index %d out of range\r\n", index);
    return;
  }

  if (pdTRUE != xQueueSend(requestQueueHandle, &profile[index], 0)) {
    printf("error: queue full\r\n");
  }
}

static bool curr_is_test_profile(void) {
  return CURR_PROFILE.a.freq >= A_FREQ_FOR_TEST;
}

/*
 * Freq
 * 50,000     10      48      480
 * 100,000    10      24      240
 * 200,000    10      12      120
 * 500,000    10      4.8     48
 */

/**
 * @brief Calculate required ADC sample count for whole cycles
 * @param frequency Signal frequency in Hz (50,000 to 500,000)
 * @param num_cycles Number of complete cycles to sample
 * @retval Required sample count
 */
uint32_t Calculate_ADC_Sample_Count(uint32_t frequency, uint32_t num_cycles) {
  const uint32_t SAMPLING_RATE = 2400000; // 2.4 MHz

  // Calculate samples for exact number of cycles
  // samples = (sampling_rate * num_cycles) / frequency
  uint32_t total_samples = (SAMPLING_RATE * num_cycles) / frequency;

  return total_samples;
}

static int measure_hv_vpp(uint32_t freq, uint32_t level, TickType_t stab_delay,
                          bool print) {

  uint32_t sample_count = Calculate_ADC_Sample_Count(freq, 20);

  DAC_SetOutput_Percent(0);
  vTaskDelay(100);

  DDS_Start(freq, false);
  DAC_SetOutput_Percent(level);

  // stabilize
  vTaskDelay(stab_delay);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, sample_count);
  while (HAL_DMA_GetState(hadc1.DMA_Handle) != HAL_DMA_STATE_READY)
    ;
  HAL_ADC_Stop_DMA(&hadc1);

  int adc_sum = 0;
  for (int i = 0; i < sample_count; i++) {
    adc_sum += adc_dma_buf[i];
  }

  float adc_avg = ((float)adc_sum) / sample_count;

  float sum_of_squares = 0;
  for (int i = 0; i < sample_count; i++) {
    float amp = (float)adc_dma_buf[i] - adc_avg;
    sum_of_squares += amp * amp;
  }

  float rms_adc = sqrtf(sum_of_squares / sample_count);
  float Vmid = (adc_avg / 4095.0) * 3.3;
  float Vrms = (rms_adc / 4095.0) * 3.3;
  float Vpp = Vrms * 2.0 * sqrtf(2.0);

  int hv_vpp = (int)(Vpp * 40 * 1000);

  if (print) {
    printf("measure_hv_vpp: freq: %lu, sample_count: %lu, Vmid: %d mV, Vrms: "
           "%d mV, Vpp: %d mV, hv_vpp: %d mV\r\n",
           freq, sample_count, (int)(Vmid * 1000), (int)(Vrms * 1000),
           (int)(Vpp * 1000), hv_vpp);
  }

  return hv_vpp;
}

void StartProfileTask(void const *argument) {

  HAL_StatusTypeDef status;
  uint32_t dur;
  uint32_t test = DDS_FreqReg(200000);

  printf("\r\n\r\n---- test freq reg ----\r\n");
  printf("200KHz, hi reg 16bit is 0x%04x\r\n", (uint16_t)(test >> 16));
  printf("200KHz, lo reg 16bit is 0x%04x\r\n", (uint16_t)test);

  printf("\r\n\r\n---- ttf boot ---- \r\n");
  // Add this in your FreeRTOS task or after MX_SPI1_Init():

  status = load_profiles();
  if (status == HAL_OK) {
    printf("profiles loaded from flash\r\n");
  } else {
    printf("no profiles stored in flash\r\n");
  }

  DAC_Start();
  DAC_SetOutput_Percent(90);
  DDS_Start(50000, true);

  for (;;) {
    vTaskDelay(1000);
  }

entry_point:

  CURR_PROFILE = NEXT_PROFILE;

  if (!curr_is_test_profile()) {

    DDS_Start(CURR_PROFILE.a.freq, false);

    for (;;) {

      // optotriac_update(&CURR_PROFILE.a);
      DAC_SetOutput_Percent(CURR_PROFILE.a.level);

      dur = CURR_PROFILE.a.duration * 1000;
      if (dur == 0)
        dur = portMAX_DELAY;

      if (pdTRUE == xQueueReceive(requestQueueHandle, &NEXT_PROFILE, dur)) {
        printf("goto from a\r\n");
        goto entry_point;
      }

      // optotriac_update(&CURR_PROFILE.b);
      DAC_SetOutput_Percent(CURR_PROFILE.b.level);

      dur = CURR_PROFILE.b.duration * 1000;
      if (dur == 0)
        dur = portMAX_DELAY;

      if (pdTRUE == xQueueReceive(requestQueueHandle, &NEXT_PROFILE, dur)) {
        printf("goto from b\r\n");
        goto entry_point;
      }
    }
  }

  if (CURR_PROFILE.a.freq == 0xFFFFFFFF) {

    int freqArg = CURR_PROFILE.a.duration;
    int levelArg = CURR_PROFILE.a.level;

    if (freqArg == 0 && levelArg == 0) {
      for (int freq = 50000, i = 0; freq <= 500000; freq += 50000, i++) {
        for (int level = 10, j = 0; level <= 100; level += 10, j++) {
          printf("freq: %d, level: %d, Vpp: %d\r\n", freq, level,
                 measure_hv_vpp(freq, level, 100, false));
          vTaskDelay(500);
        }
      }

      NEXT_PROFILE = STOP_PROFILE;
      goto entry_point;
    }

    if (freqArg >= 50000 && freqArg <= 500000 && levelArg >= 10 &&
        levelArg <= 100) {
      printf("freq: %d, level: %d, Vpp: %d -- hold\r\n", freqArg, levelArg,
             measure_hv_vpp(freqArg, levelArg, 100, false));

      if (pdTRUE ==
          xQueueReceive(requestQueueHandle, &NEXT_PROFILE, portMAX_DELAY)) {
        goto entry_point;
      }
    }
  }

  NEXT_PROFILE = STOP_PROFILE;
  goto entry_point;
}

// STM32F405 Flash memory is organized in sectors of varying sizes
// STM32F405 has 1MB Flash (0x100000 bytes)
// Flash starts at 0x08000000
// Sector sizes:
// Sectors 0-3: 16KB each
// Sectors 4: 64KB
// Sectors 5-11: 128KB each
// Total 1MB (0x100000 bytes)

#define FLASH_SIZE 0x00100000U // 1MB
#define FLASH_ADDR_BASE 0x08000000UL
// #define FLASH_SECTOR_3
#define FLASH_SECTOR_3_SIZE 0x00004000U // 16KB
#define FLASH_SECTOR_3_ADDR (FLASH_ADDR_BASE + FLASH_SECTOR_3_SIZE * 3UL)

static HAL_StatusTypeDef erase_sector_3(void) {
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  HAL_StatusTypeDef status;

  /* Fill EraseInit structure */
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange =
      FLASH_VOLTAGE_RANGE_3;               // Voltage range 2.7V to 3.6V
  EraseInitStruct.Sector = FLASH_SECTOR_3; // Sector 3
  EraseInitStruct.NbSectors = 1;

  status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
  return status;
}

/**
 * this value is 12 in previous version, in which sizeof(profile_t) is 48;
 * now it is sizeof(profile_v2_t) / 4 = 32 / 4 = 8
 */
#define SIZE_OF_PROFILE_IN_WORD 8
#define SIZE_OF_PROFILES (sizeof(profile_v2_t) * NUM_OF_PROFILES)
#define SIZE_OF_PROFILES_IN_WORD (SIZE_OF_PROFILES / 4)

#define DEBUG_WRITING_EVERY_N_WORDS 8

static HAL_StatusTypeDef save_profiles(void) {
  HAL_StatusTypeDef status;

  // Debug output
  printf("Profile size: %u bytes (%u words)\r\n", sizeof(profile_v2_t),
         SIZE_OF_PROFILE_IN_WORD);
  printf("Total profiles: %u\r\n", NUM_OF_PROFILES);
  printf("Start address: 0x%08lX\r\n", (FLASH_SECTOR_3_ADDR));

  // Calculate CRC
  uint32_t crc =
      HAL_CRC_Calculate(&hcrc, (uint32_t *)profile, SIZE_OF_PROFILES_IN_WORD);

  /* Unlock the Flash to enable the flash control register access */
  HAL_FLASH_Unlock();

  // Clear all error flags
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR |
                         FLASH_FLAG_PGSERR);

  status = erase_sector_3();
  if (status != HAL_OK) {
    printf("error: failed to erase sector 3\r\n");
    HAL_FLASH_Lock();
    return status;
  }

  vTaskDelay(100);

  // Write profile
  uint32_t *source_ptr = (uint32_t *)profile;
  uint32_t write_address;

  for (int i = 0; i < SIZE_OF_PROFILES_IN_WORD; i++) {
    write_address = FLASH_SECTOR_3_ADDR + (i * sizeof(uint32_t));

#if DEBUG_WRITING_EVERY_N_WORDS
    // Print debug info every N words, just print the single word?
    if (i % (DEBUG_WRITING_EVERY_N_WORDS) == 0) {
      printf("Writing word %d (0x%08lX) to address 0x%08lX\r\n", i,
             source_ptr[i], write_address);
    }
#endif

    status =
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, write_address, source_ptr[i]);

    if (status != HAL_OK) {
      printf("error: failed to write word at index %d (addr: 0x%08lX, value: "
             "0x%08lX)\r\n",
             i, write_address, source_ptr[i]);
      printf("Flash SR: 0x%08lX\r\n", FLASH->SR);
      HAL_FLASH_Lock();
      return status;
    }

    vTaskDelay(4);
  }

  // Write CRC
  write_address = FLASH_SECTOR_3_ADDR + SIZE_OF_PROFILES;
  printf("Writing CRC (0x%08lX) to address 0x%08lX\r\n", crc, write_address);

  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, write_address, crc);
  if (status != HAL_OK) {
    printf("error: failed to write CRC\r\n");
    HAL_FLASH_Lock();
    return status;
  }

  HAL_FLASH_Lock();
  vTaskDelay(100);

  // Verify first few words were written correctly
  printf("Verification:\r\n");
  uint32_t *verify_ptr = (uint32_t *)FLASH_SECTOR_3_ADDR;
  bool all_match = true;
  for (int i = 0; i < SIZE_OF_PROFILES_IN_WORD; i++) {

    if (verify_ptr[i] != source_ptr[i]) {
      printf("Word %d: Flash=0x%08lX, Original=0x%08lX %s\r\n", i,
             verify_ptr[i], source_ptr[i], "MISMATCH");
      all_match = false;
    }
  }

  if (all_match) {
    printf("ALL WORDS MATCH.\r\n");
  }

  printf("CRC: Flash=0x%08lX, Original=0x%08lX %s\r\n",
         verify_ptr[SIZE_OF_PROFILES_IN_WORD], crc,
         verify_ptr[SIZE_OF_PROFILES_IN_WORD] == crc ? "MATCH" : "MISMATCH");

  return status;
}

static HAL_StatusTypeDef load_profiles(void) {
  static profile_v2_t _profile[NUM_OF_PROFILES];

  for (int i = 0; i < NUM_OF_PROFILES; i++) {
    for (int j = 0; j < SIZE_OF_PROFILE_IN_WORD; j++) {
      _profile[i].word[j] =
          *((__IO uint32_t *)(FLASH_SECTOR_3_ADDR + i * sizeof(profile_v2_t) +
                              j * 4));
    }
  }

  uint32_t stored_crc =
      *(__IO uint32_t *)(FLASH_SECTOR_3_ADDR + SIZE_OF_PROFILES);
  uint32_t calculated_crc =
      HAL_CRC_Calculate(&hcrc, (uint32_t *)_profile, SIZE_OF_PROFILES_IN_WORD);

  if (stored_crc != calculated_crc) {
    return HAL_ERROR;
  } else {
    for (int i = 0; i < NUM_OF_PROFILES; i++) {
      profile[i] = _profile[i];
    }

    return HAL_OK;
  }
}
