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

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "profile.h"
#include "tca9555.h"

#define STOP_PROFILE profile[STOP_PROFILE_INDEX]
#define DDBF_PROFILE                                                           \
  profile[DDBF_PROFILE_INDEX] // deadbeef, special profile, not used
#define CURR_PROFILE profile[CURR_PROFILE_INDEX]
#define NEXT_PROFILE profile[NEXT_PROFILE_INDEX]

extern osMessageQId requestQueueHandle;

static profile_t profile[NUM_OF_ALL_PROFILES] = {
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
    {}, // 17, deadbeef is not used in this application
        //     it is used in original multipads project for implement the blink
        //     function. it is designed to be a special profile responds to STOP
        //     key. if there are profile key already pressed, aka, the switches
        //     are working, then pressing STOP key will stop action. if there is
        //     no profile key already pressed, pressing STOP key will trigger
        //     blink, in which case deadbeef profile is sent.
    {}, // 18, current
    {}, // 19, next
};

extern CRC_HandleTypeDef hcrc;

static HAL_StatusTypeDef save_profiles(void);
static HAL_StatusTypeDef load_profiles(void);

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

static void print_allpads_str(allpads_t *pads, char str[54]) {
  for (int i = 0; i < 3; i++) {
    str[i * 18 + 0] = cfg2char(pads->row[i].a_top);
    str[i * 18 + 1] = cfg2char(pads->row[i].a_lft);
    str[i * 18 + 2] = cfg2char(pads->row[i].a_mid);
    str[i * 18 + 3] = cfg2char(pads->row[i].a_rgt);
    str[i * 18 + 4] = cfg2char(pads->row[i].a_bot);
    str[i * 18 + 5] = ',';
    str[i * 18 + 6] = cfg2char(pads->row[i].b_top);
    str[i * 18 + 7] = cfg2char(pads->row[i].b_lft);
    str[i * 18 + 8] = cfg2char(pads->row[i].b_mid);
    str[i * 18 + 9] = cfg2char(pads->row[i].b_rgt);
    str[i * 18 + 10] = cfg2char(pads->row[i].b_bot);
    str[i * 18 + 11] = ',';
    str[i * 18 + 12] = cfg2char(pads->row[i].c_top);
    str[i * 18 + 13] = cfg2char(pads->row[i].c_lft);
    str[i * 18 + 14] = cfg2char(pads->row[i].c_mid);
    str[i * 18 + 15] = cfg2char(pads->row[i].c_rgt);
    str[i * 18 + 16] = cfg2char(pads->row[i].c_bot);
    str[i * 18 + 17] = i == 2 ? 0 : ';';
  }
}

void print_profile(int i) {
  static char str[54];

  if (i >= NUM_OF_ALL_PROFILES) {
    return;
  }

  print_allpads_str(&profile[i].a.pads, str);
  printf("Profile #%02d phase a: %s in %d seconds at %d volts and %lu Hz\r\n",
         i, str, profile[i].a.duration, profile[i].a.level, profile[i].a.freq);

  print_allpads_str(&profile[i].b.pads, str);
  printf("            phase b: %s in %d seconds at %d volts and %lu Hz\r\n",
         str, profile[i].b.duration, profile[i].b.level, profile[i].b.freq);
}

profile_t get_profile(int index) {
  profile_t prfl = {0};

  if (index > -1 && index < 16) {
    return profile[index];
  }
  return prfl;
}

void set_profile_phase(int profile_index, int phase_index,
                       const phase_t *phase) {
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

#ifdef TCA9555
/*
 * @brief apply pad configuration to ports
 */
static void padscfg_to_portcfg(allpads_t *pads, uint8_t port[6][2]) {
  // @format:off
  static const netmap_t nm[46] = {
      {},
      {0, 1, {0, 1 << 7, 1 << 6}}, // P01
      {0, 1, {0, 1 << 5, 1 << 4}}, // P02
      {0, 1, {0, 1 << 3, 1 << 2}}, // P03
      {0, 1, {0, 1 << 1, 1 << 0}}, // P04
      {0, 0, {0, 1 << 6, 1 << 7}}, // P05
      {0, 0, {0, 1 << 4, 1 << 5}}, // P06
      {0, 0, {0, 1 << 2, 1 << 3}}, // P07
      {0, 0, {0, 1 << 0, 1 << 1}}, // P08

      {1, 1, {0, 1 << 7, 1 << 6}}, // P09
      {1, 1, {0, 1 << 5, 1 << 4}}, // P10
      {1, 1, {0, 1 << 3, 1 << 2}}, // P11
      {1, 1, {0, 1 << 1, 1 << 0}}, // P12
      {1, 0, {0, 1 << 6, 1 << 7}}, // P13
      {1, 0, {0, 1 << 4, 1 << 5}}, // P14
      {1, 0, {0, 1 << 2, 1 << 3}}, // P15

      {2, 1, {0, 1 << 7, 1 << 6}}, // P16
      {2, 1, {0, 1 << 5, 1 << 4}}, // P17
      {2, 1, {0, 1 << 3, 1 << 2}}, // P18
      {2, 1, {0, 1 << 1, 1 << 0}}, // P19
      {2, 0, {0, 1 << 6, 1 << 7}}, // P20
      {2, 0, {0, 1 << 4, 1 << 5}}, // P21
      {2, 0, {0, 1 << 2, 1 << 3}}, // P22
      {2, 0, {0, 1 << 0, 1 << 1}}, // P23

      {3, 1, {0, 1 << 7, 1 << 6}}, // P24
      {3, 1, {0, 1 << 5, 1 << 4}}, // P25
      {3, 1, {0, 1 << 3, 1 << 2}}, // P26
      {3, 1, {0, 1 << 1, 1 << 0}}, // P27
      {3, 0, {0, 1 << 6, 1 << 7}}, // P28
      {3, 0, {0, 1 << 4, 1 << 5}}, // P29
      {3, 0, {0, 1 << 2, 1 << 3}}, // P30

      {4, 1, {0, 1 << 7, 1 << 6}}, // P31
      {4, 1, {0, 1 << 5, 1 << 4}}, // P32
      {4, 1, {0, 1 << 3, 1 << 2}}, // P33
      {4, 1, {0, 1 << 1, 1 << 0}}, // P34
      {4, 0, {0, 1 << 6, 1 << 7}}, // P35
      {4, 0, {0, 1 << 4, 1 << 5}}, // P36
      {4, 0, {0, 1 << 2, 1 << 3}}, // P37
      {4, 0, {0, 1 << 0, 1 << 1}}, // P38

      {5, 1, {0, 1 << 7, 1 << 6}}, // P39
      {5, 1, {0, 1 << 5, 1 << 4}}, // P40
      {5, 1, {0, 1 << 3, 1 << 2}}, // P41
      {5, 1, {0, 1 << 1, 1 << 0}}, // P42
      {5, 0, {0, 1 << 6, 1 << 7}}, // P43
      {5, 0, {0, 1 << 4, 1 << 5}}, // P44
      {5, 0, {0, 1 << 2, 1 << 3}}, // P45
  };
  // @format:on

  memset(port, 0, 12);

  /*
   * netname according to pcb and sch.
   *
   * 44,45,43,41,42;29,30,28,26,27;14,15,13,11,12
   * 39,40,38,36,37;24,25,23,21,22;09,10,08,06,07
   * 34,35,33,31,32;19,20,18,16,17;04,05,03,01,02
   */

#define PADCFG(num, cfg) port[nm[num].addr][nm[num].port] |= nm[num].mask[cfg]

  PADCFG(44, pads->row[0].a_top);
  PADCFG(45, pads->row[0].a_lft);
  PADCFG(43, pads->row[0].a_mid);
  PADCFG(41, pads->row[0].a_rgt);
  PADCFG(42, pads->row[0].a_bot);

  PADCFG(29, pads->row[0].b_top);
  PADCFG(30, pads->row[0].b_lft);
  PADCFG(28, pads->row[0].b_mid);
  PADCFG(26, pads->row[0].b_rgt);
  PADCFG(27, pads->row[0].b_bot);

  PADCFG(14, pads->row[0].c_top);
  PADCFG(15, pads->row[0].c_lft);
  PADCFG(13, pads->row[0].c_mid);
  PADCFG(11, pads->row[0].c_rgt);
  PADCFG(12, pads->row[0].c_bot); // here

  PADCFG(39, pads->row[1].a_top);
  PADCFG(40, pads->row[1].a_lft);
  PADCFG(38, pads->row[1].a_mid);
  PADCFG(36, pads->row[1].a_rgt);
  PADCFG(37, pads->row[1].a_bot);

  PADCFG(24, pads->row[1].b_top);
  PADCFG(25, pads->row[1].b_lft);
  PADCFG(23, pads->row[1].b_mid);
  PADCFG(21, pads->row[1].b_rgt);
  PADCFG(22, pads->row[1].b_bot);

  PADCFG(9, pads->row[1].c_top);
  PADCFG(10, pads->row[1].c_lft);
  PADCFG(8, pads->row[1].c_mid);
  PADCFG(6, pads->row[1].c_rgt);
  PADCFG(7, pads->row[1].c_bot);

  PADCFG(34, pads->row[2].a_top);
  PADCFG(35, pads->row[2].a_lft);
  PADCFG(33, pads->row[2].a_mid);
  PADCFG(31, pads->row[2].a_rgt);
  PADCFG(32, pads->row[2].a_bot);

  PADCFG(19, pads->row[2].b_top);
  PADCFG(20, pads->row[2].b_lft);
  PADCFG(18, pads->row[2].b_mid);
  PADCFG(16, pads->row[2].b_rgt);
  PADCFG(17, pads->row[2].b_bot);

  PADCFG(4, pads->row[2].c_top);
  PADCFG(5, pads->row[2].c_lft);
  PADCFG(3, pads->row[2].c_mid);
  PADCFG(1, pads->row[2].c_rgt);
  PADCFG(2, pads->row[2].c_bot);

#undef PADCFG
}

#endif

extern const char *bit_rep[16];

void do_profile(int index) {
  //  if (index >= 0 && index < 16)
  //  {
  //    if (pdTRUE != xQueueSend(requestQueueHandle, &profile[index], 0))
  //    {
  //      printf("error: queue full\r\n");
  //    }
  //  }
  //  else
  //  {
  //    if (pdTRUE != xQueueSend(requestQueueHandle, &STOP_PROFILE, 0))
  //    {
  //      printf("error: queue full\r\n");
  //    }
  //  }
  if (index < 0 || index > LAST_PROFILE_INDEX) {
    printf("error: do_profile, index %d out of range\r\n", index);
    return;
  }

  if (pdTRUE != xQueueSend(requestQueueHandle, &profile[index], 0)) {
    printf("error: queue full\r\n");
  }
}

// int level_to_dac_in_mv(int level)
//{
//   if (level >= 100)
//   {
//     return 0;
//   }
//
//   if (level <= 0)
//   {
//     return 700;
//   }
//
//   return (100 - level) * 7;
// }

// static void apply_padscfg(allpads_t *pads)
//{
//   uint8_t port[6][2] =
//   { 0 };
//   padscfg_to_portcfg(pads, port);
//
////  printf("port=");
////  for (int i = 0; i < 6; i++)
////  {
////    printf("%s%s,%s%s", bit_rep[port[i][1] >> 4], bit_rep[port[i][1] &
/// 0x0F], /        bit_rep[port[i][0] >> 4], bit_rep[port[i][0] & 0x0F]); / if
///(i < 5) /    { /      printf("; "); /    } /  } /  printf("\r\n");
//
//  TCA9555_UpdateOutput(port);
//}

// #define ZERO_VOLT_BEFORE_BREAK        (true)
#define DELAY_BEFORE_BREAK (1)
#define DELAY_AFTER_BREAK (0)

static void optotriac_update(phase_t *phase) {
  //  static uint8_t port[6][2];
  //  padscfg_to_portcfg(&phase->pads, port);

  // with buffer on, disable dac output directly
  // makes output to middle voltage of VREF
  DAC_Disable();

  if (DELAY_BEFORE_BREAK > 0)
    vTaskDelay(DELAY_BEFORE_BREAK);

  // break
  // TCA9555_Break();
  // or direct make
  // TCA9555_Make(port);

  if (DELAY_AFTER_BREAK > 0)
    vTaskDelay(DELAY_AFTER_BREAK);

  DAC_Enable();
  DAC_SetOutput_Percent(phase->level);
}

void StartProfileTask(void const *argument) {

  HAL_StatusTypeDef status;

  uint32_t test = DDS_FreqReg(200000);
  printf("\r\n\r\n---- test freq reg ----\r\n");
  printf("200KHz, hi reg 16bit is 0x%04x\r\n", (uint16_t)(test >> 16));
  printf("200KHz, lo reg 16bit is 0x%04x\r\n", (uint16_t)test);

  printf("\r\n\r\n---- ttf boot ---- \r\n");
  printf("version: 1.0.1-20250428\r\n");
  printf(
      "  - use TCA9555_VerifiedWriteReg to detect i2c write failure.\r\n\r\n");

  // TCA9555_Init_All();
  // TCA9555_Dump();

  status = load_profiles();
  if (status == HAL_OK) {
    printf("profiles loaded from flash\r\n");
  } else {
    printf("no profiles stored in flash\r\n");
  }

  // DDS_Start(100000, false);
  // vTaskDelay(100);

  DAC_Start();
  DAC_SetOutput_Percent(0);

  // vTaskDelay(100);

entry_point:

  CURR_PROFILE = NEXT_PROFILE;
  DDS_Start(CURR_PROFILE.a.freq, false);

  vTaskDelay(100);

  for (;;) {
    uint32_t dur;

    optotriac_update(&CURR_PROFILE.a);

    dur = CURR_PROFILE.a.duration * 1000;
    if (dur == 0)
      dur = portMAX_DELAY;

    if (pdTRUE == xQueueReceive(requestQueueHandle, &NEXT_PROFILE, dur)) {
      printf("goto from a\r\n");
      goto entry_point;
    }

    optotriac_update(&CURR_PROFILE.b);

    dur = CURR_PROFILE.b.duration * 1000;
    if (dur == 0)
      dur = portMAX_DELAY;

    if (pdTRUE == xQueueReceive(requestQueueHandle, &NEXT_PROFILE, dur)) {
      printf("goto from b\r\n");
      goto entry_point;
    }
  }
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

#define SIZE_OF_PROFILE_IN_WORD 12
#define SIZE_OF_PROFILES (sizeof(profile_t) * NUM_OF_PROFILES)
#define SIZE_OF_PROFILES_IN_WORD (SIZE_OF_PROFILES / 4)

#define DEBUG_WRITING_EVERY_N_WORDS 12

static HAL_StatusTypeDef save_profiles(void) {
  HAL_StatusTypeDef status;

  // Debug output
  printf("Profile size: %u bytes (%u words)\r\n", sizeof(profile_t),
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
    // Print debug info every 10 words
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

#if 0

/*
 * write profile_t[16] to flash. sizeof(profile_t) == 40.
 */
static HAL_StatusTypeDef save_profiles(void)
{
  HAL_StatusTypeDef status;

  // calculate crc
  uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) profile,
      SIZE_OF_PROFILES_IN_WORD);

  /* Unlock the Flash to enable the flash control register access */
  HAL_FLASH_Unlock();

  status = erase_sector_3();
  if (status != HAL_OK)
  {
    printf("error: failed to erase sector 3\r\n");
    HAL_FLASH_Lock();
    return status;
  }

  vTaskDelay(500);

  // write profile
  for (int i = 0; i < NUM_OF_PROFILES; i++)
  {
    for (int j = 0; j < SIZE_OF_PROFILE_IN_WORD; j++)
    {
      status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
          (FLASH_SECTOR_3_ADDR + i * SIZE_OF_PROFILE_IN_WORD + j * 4), profile[i].word[j]);

      if (status != HAL_OK)
      {
        printf("error: failed to write profile[%d].word[%d]\r\n", i, j);
        HAL_FLASH_Lock();
        return status;
      }

      vTaskDelay(40);
    }
  }

  // write crc
  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
      (FLASH_SECTOR_3_ADDR + SIZE_OF_PROFILES), crc);

  if (status != HAL_OK)
  {
    printf("error: failed to write CRC\r\n");
    HAL_FLASH_Lock();
    return status;
  }

  HAL_FLASH_Lock();
  vTaskDelay(100);
  return status;
}

#endif

static HAL_StatusTypeDef load_profiles(void) {
  static profile_t _profile[NUM_OF_PROFILES];

  for (int i = 0; i < NUM_OF_PROFILES; i++) {
    for (int j = 0; j < SIZE_OF_PROFILE_IN_WORD; j++) {
      _profile[i].word[j] = *((__IO uint32_t *)(FLASH_SECTOR_3_ADDR +
                                                i * sizeof(profile_t) + j * 4));
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
