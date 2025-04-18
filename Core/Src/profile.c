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
#include "main.h"
#include "profile.h"
#include "tca9555.h"

#define STOP_PROFILE            profile[16]
#define DDBF_PROFILE            profile[17] // deadbeef, special profile, not used
#define CURR_PROFILE            profile[18]
#define NEXT_PROFILE            profile[19]

extern osMessageQId requestQueueHandle;

static profile_t profile[20] =
{
  { },  // 0
  { },  // 1
  { },  // 2
  { },  // 3
  { },  // 4
  { },  // 5
  { },  // 6
  { },  // 7
  { },  // 8
  { },  // 9
  { },  // 10
  { },  // 11
  { },  // 12
  { },  // 13
  { },  // 14
  { },  // 15
  { },  // 16, stop
  { },  // 17, deadbeef is not used in this application
        //     it is used in original multipads project for implement the blink function.
        //     it is designed to be a special profile responds to STOP key.
        //     if there are profile key already pressed, aka, the switches are working, then
        //     pressing STOP key will stop action.
        //     if there is no profile key already pressed, pressing STOP key will
        //     trigger blink, in which case deadbeef profile is sent.
  { },  // 18, current
  { },  // 19, next
    };

extern CRC_HandleTypeDef hcrc;

static HAL_StatusTypeDef save_profiles(void);
static HAL_StatusTypeDef load_profiles(void);

/* Private function prototypes -----------------------------------------------*/


/* Private user code ---------------------------------------------------------*/



/* Public user code ---------------------------------------------------------*/

static char cfg2char(unsigned int cfg)
{
  if (cfg == 0)
  {
    return '0';
  }
  else if (cfg == 1)
  {
    return '1';
  }
  else if (cfg == 2)
  {
    return '2';
  }
  else
  {
    return '*';
  }
}

static void print_allpads_str(allpads_t *pads, char str[54])
{
  for (int i = 0; i < 3; i++)
  {
    str[i * 18 + 0]  = cfg2char(pads->row[i].a_top);
    str[i * 18 + 1]  = cfg2char(pads->row[i].a_lft);
    str[i * 18 + 2]  = cfg2char(pads->row[i].a_mid);
    str[i * 18 + 3]  = cfg2char(pads->row[i].a_rgt);
    str[i * 18 + 4]  = cfg2char(pads->row[i].a_bot);
    str[i * 18 + 5]  = ',';
    str[i * 18 + 6]  = cfg2char(pads->row[i].b_top);
    str[i * 18 + 7]  = cfg2char(pads->row[i].b_lft);
    str[i * 18 + 8]  = cfg2char(pads->row[i].b_mid);
    str[i * 18 + 9]  = cfg2char(pads->row[i].b_rgt);
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

void print_profile(int i)
{
  static char str[54];

  if (i >= 16)
  {
    return;
  }

  print_allpads_str(&profile[i].a.pads, str);
  printf("Profile #%02d phase a: %s in %d seconds at %d volts\r\n", i, str,
      profile[i].a.duration, profile[i].a.level);

  print_allpads_str(&profile[i].b.pads, str);
  printf("            phase b: %s in %d seconds at %d volts\r\n", str,
      profile[i].b.duration, profile[i].b.level);
}

profile_t get_profile(int index)
{
  profile_t prfl = {0};

	if (index > -1 && index < 16)
	{
		return profile[index];
	}
	return prfl;
}

void set_profile_phase(int profile_index, int phase_index, phase_t *phase)
{
  if (phase_index == 0)
  {
    profile[profile_index].a = *phase;
  }
  else
  {
    profile[profile_index].b = *phase;
  }

  // save_profiles();
}

#if 0

GPIO_PinState pin_state(uint32_t cfg, int bitpos)
{
	return (cfg & (1 << bitpos)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void update_switch(int port_index, int pin_index, GPIO_PinState c_state, GPIO_PinState s_state)
{
	if (s_state == GPIO_PIN_SET)
	{
		HAL_GPIO_WritePin(c_port[port_index][pin_index], c_pin[port_index][pin_index], c_state);
		HAL_GPIO_WritePin(s_port[port_index][pin_index], s_pin[port_index][pin_index], s_state);
	}
	else
	{
		HAL_GPIO_WritePin(s_port[port_index][pin_index], s_pin[port_index][pin_index], s_state);
		HAL_GPIO_WritePin(c_port[port_index][pin_index], c_pin[port_index][pin_index], c_state);
	}
}

void update_all_switches(uint32_t pg_cfg[4])
{
	for (int port = 0; port < 4; port++)
	{
		for (int pin = 0; pin < 9; pin++)
		{
			GPIO_PinState c_state = pg_cfg[port] & (1 << (2 * pin + 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
			GPIO_PinState s_state = pg_cfg[port] & (1 << (2 * pin + 1)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
			update_switch(port, pin, c_state, s_state);
		}
	}
}

void do_profile_by_key(int key)
{
	static bool non_zero_key_ever_pressed = false;

	snprintf(long_buf, 256, "key %u pressed", key);
	print_line(long_buf);

	if (key == 0)
	{
		if (non_zero_key_ever_pressed)
		{
			if (pdTRUE != xQueueSend(requestQueueHandle, &STOP_PROFILE, 0))
			{
				print_line("error: queue full");
			}
		}
		else
		{
			if (pdTRUE != xQueueSend(requestQueueHandle, &DDBF_PROFILE, 0))
			{
				print_line("error: queue full");
			}
		}
	}
	else if (key > 0 && key < 10)
	{
		non_zero_key_ever_pressed = true;
		if (pdTRUE != xQueueSend(requestQueueHandle, &profile[key - 1], 0))
		{
			print_line("error: queue full");
		}
	}
}
#endif


typedef struct
{
  uint8_t addr;     // 0-6
  uint8_t port;     // 0 or 1
  uint8_t mask[3];  // see netmap initializer
} netmap_t;

/*
 * @brief apply pad configuration to ports
 */
static void padscfg_to_portcfg(allpads_t * pads, uint8_t port[6][2])
{
  // @format:off
  static const netmap_t nm[46] =
  {
    { },
    { 0, 1, { 0, 1 << 7, 1 << 6 } }, // P01
    { 0, 1, { 0, 1 << 5, 1 << 4 } }, // P02
    { 0, 1, { 0, 1 << 3, 1 << 2 } }, // P03
    { 0, 1, { 0, 1 << 1, 1 << 0 } }, // P04
    { 0, 0, { 0, 1 << 6, 1 << 7 } }, // P05
    { 0, 0, { 0, 1 << 4, 1 << 5 } }, // P06
    { 0, 0, { 0, 1 << 2, 1 << 3 } }, // P07
    { 0, 0, { 0, 1 << 0, 1 << 1 } }, // P08

    { 1, 1, { 0, 1 << 7, 1 << 6 } }, // P09
    { 1, 1, { 0, 1 << 5, 1 << 4 } }, // P10
    { 1, 1, { 0, 1 << 3, 1 << 2 } }, // P11
    { 1, 1, { 0, 1 << 1, 1 << 0 } }, // P12
    { 1, 0, { 0, 1 << 6, 1 << 7 } }, // P13
    { 1, 0, { 0, 1 << 4, 1 << 5 } }, // P14
    { 1, 0, { 0, 1 << 2, 1 << 3 } }, // P15

    { 0, 1, { 0, 1 << 7, 1 << 6 } }, // P16
    { 0, 1, { 0, 1 << 5, 1 << 4 } }, // P17
    { 0, 1, { 0, 1 << 3, 1 << 2 } }, // P18
    { 0, 1, { 0, 1 << 1, 1 << 0 } }, // P19
    { 0, 0, { 0, 1 << 6, 1 << 7 } }, // P20
    { 0, 0, { 0, 1 << 4, 1 << 5 } }, // P21
    { 0, 0, { 0, 1 << 2, 1 << 3 } }, // P22
    { 0, 0, { 0, 1 << 0, 1 << 1 } }, // P23

    { 1, 1, { 0, 1 << 7, 1 << 6 } }, // P24
    { 1, 1, { 0, 1 << 5, 1 << 4 } }, // P25
    { 1, 1, { 0, 1 << 3, 1 << 2 } }, // P26
    { 1, 1, { 0, 1 << 1, 1 << 0 } }, // P27
    { 1, 0, { 0, 1 << 6, 1 << 7 } }, // P28
    { 1, 0, { 0, 1 << 4, 1 << 5 } }, // P29
    { 1, 0, { 0, 1 << 2, 1 << 3 } }, // P30

    { 0, 1, { 0, 1 << 7, 1 << 6 } }, // P31
    { 0, 1, { 0, 1 << 5, 1 << 4 } }, // P32
    { 0, 1, { 0, 1 << 3, 1 << 2 } }, // P33
    { 0, 1, { 0, 1 << 1, 1 << 0 } }, // P34
    { 0, 0, { 0, 1 << 6, 1 << 7 } }, // P35
    { 0, 0, { 0, 1 << 4, 1 << 5 } }, // P36
    { 0, 0, { 0, 1 << 2, 1 << 3 } }, // P37
    { 0, 0, { 0, 1 << 0, 1 << 1 } }, // P38

    { 1, 1, { 0, 1 << 7, 1 << 6 } }, // P39
    { 1, 1, { 0, 1 << 5, 1 << 4 } }, // P40
    { 1, 1, { 0, 1 << 3, 1 << 2 } }, // P41
    { 1, 1, { 0, 1 << 1, 1 << 0 } }, // P42
    { 1, 0, { 0, 1 << 6, 1 << 7 } }, // P43
    { 1, 0, { 0, 1 << 4, 1 << 5 } }, // P44
    { 1, 0, { 0, 1 << 2, 1 << 3 } }, // P45
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
  PADCFG(12, pads->row[0].c_bot);

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

  PADCFG( 9, pads->row[1].c_top);
  PADCFG(10, pads->row[1].c_lft);
  PADCFG( 8, pads->row[1].c_mid);
  PADCFG( 6, pads->row[1].c_rgt);
  PADCFG( 7, pads->row[1].c_bot);

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

  PADCFG( 4, pads->row[2].c_top);
  PADCFG( 5, pads->row[2].c_lft);
  PADCFG( 3, pads->row[2].c_mid);
  PADCFG( 1, pads->row[2].c_rgt);
  PADCFG( 2, pads->row[2].c_bot);

#undef PADCFG
}

static void apply_padscfg(allpads_t * pads)
{
  uint8_t port[6][2] = {0};
  padscfg_to_portcfg(pads, port);
  TCA9555_UpdateOutput(port);
}

void do_profile(int index)
{
  if (index >= 0 && index < 16)
  {
    if (pdTRUE != xQueueSend(requestQueueHandle, &profile[index], 0))
    {
      printf("error: queue full\r\n");
    }
  }
  else
  {
    if (pdTRUE != xQueueSend(requestQueueHandle, &STOP_PROFILE, 0))
    {
      printf("error: queue full\r\n");
    }
  }
}

int level_to_dac_in_mv(int level)
{
  if (level >= 100)
  {
    return 0;
  }

  if (level <= 0)
  {
    return 700;
  }

  return (100 - level) * 7;
}

void StartProfileTask(void const *argument)
{
  HAL_StatusTypeDef status;
  int dac_in_mv;
  printf("\r\n\r\n---- boot ----\r\n");

  vTaskDelay(100);

  status = load_profiles();
  if (status == HAL_OK)
  {
    printf("profiles loaded from flash\r\n");
  }
  else
  {
    printf("no profiles stored in flash\r\n");
  }

  DDS_Start();
  DAC_Start();
  DAC_Update(630);  // 10%

  vTaskDelay(100);

entry_point:
  CURR_PROFILE= NEXT_PROFILE;

  for (;;)
  {
    uint32_t dur;

    apply_padscfg(&CURR_PROFILE.a.pads);
    dac_in_mv = level_to_dac_in_mv(CURR_PROFILE.a.level);
    DAC_Update(dac_in_mv);
    dur = CURR_PROFILE.a.duration * 1000;
    if (dur == 0)
      dur = portMAX_DELAY;

    if (pdTRUE == xQueueReceive(requestQueueHandle, &NEXT_PROFILE, dur))
    {
      goto entry_point;
    }

    apply_padscfg(&CURR_PROFILE.b.pads);
    dac_in_mv = level_to_dac_in_mv(CURR_PROFILE.b.level);
    DAC_Update(level_to_dac_in_mv(CURR_PROFILE.b.level));
    dur = CURR_PROFILE.b.duration *1000;
    if (dur == 0)
      dur = portMAX_DELAY;

    if (pdTRUE == xQueueReceive(requestQueueHandle, &NEXT_PROFILE, dur))
    {
      goto entry_point;
    }
  }
/*
entry_point:
  CURR_PROFILE = NEXT_PROFILE;

  for (;;)
  {
    uint32_t dur;

    update_all_switches(CURR_PROFILE.pgcfg_a);
    dur = CURR_PROFILE.duration_a_sec * 1000;
    if (dur == 0)
      dur = portMAX_DELAY;

    if (pdTRUE == xQueueReceive(requestQueueHandle, &NEXT_PROFILE, dur))
    {
      goto entry_point;
    }

    update_all_switches(CURR_PROFILE.pgcfg_b);
    dur = CURR_PROFILE.duration_b_sec * 1000;
    if (dur == 0)
      dur = portMAX_DELAY;

    if (pdTRUE == xQueueReceive(requestQueueHandle, &NEXT_PROFILE, dur))
    {
      goto entry_point;
    }
  } */
}

// STM32F405 Flash memory is organized in sectors of varying sizes
// STM32F405 has 1MB Flash (0x100000 bytes)
// Flash starts at 0x08000000
// Sector sizes:
// Sectors 0-3: 16KB each
// Sectors 4: 64KB
// Sectors 5-11: 128KB each
// Total 1MB (0x100000 bytes)

#define FLASH_SIZE              0x00100000  // 1MB
#define FLASH_ADDR_BASE         0x08000000
// #define FLASH_SECTOR_3
#define FLASH_SECTOR_3_SIZE     0x00004000  // 16KB
#define FLASH_SECTOR_3_ADDR     (FLASH_ADDR_BASE + FLASH_SECTOR_3_SIZE * 3)

static void delay(void)
{
  vTaskDelay(50);
}

static HAL_StatusTypeDef erase_sector_3(void)
{
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  HAL_StatusTypeDef status;

  /* Fill EraseInit structure */
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3; // Voltage range 2.7V to 3.6V
  EraseInitStruct.Sector = FLASH_SECTOR_3;              // Sector 3
  EraseInitStruct.NbSectors = 1;

  status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
  return status;
}

#define NUM_OF_PROFILES                 16

#define SIZE_OF_PROFILE_IN_WORD         10
#define SIZE_OF_PROFILE_IN_DOUBLEWORD   5
#define SIZE_OF_PROFILES                (sizeof(profile_t) * NUM_OF_PROFILES)
#define SIZE_OF_PROFILES_IN_DOUBLEWORD  (SIZE_OF_PROFILES / 8)
#define SIZE_OF_PROFILES_IN_WORD        (SIZE_OF_PROFILES / 4)

/*
 * write profile_t[16] to flash. sizeof(profile_t) == 40.
 */
static HAL_StatusTypeDef save_profiles(void)
{
  HAL_StatusTypeDef status;
  uint64_t *dword = (uint64_t*) profile;

  // calculate crc
  uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) profile,
      SIZE_OF_PROFILES_IN_WORD);

  /* Unlock the Flash to enable the flash control register access */
  HAL_FLASH_Unlock();

  status = erase_sector_3();
  if (status != HAL_OK)
  {
    HAL_FLASH_Lock();
    return status;
  }

  delay();

  // write profile
  for (int i = 0; i < SIZE_OF_PROFILES_IN_DOUBLEWORD; i++)
  {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
        (FLASH_SECTOR_3_ADDR + i * sizeof(uint64_t)), dword[i]);

    if (status != HAL_OK)
    {
      HAL_FLASH_Lock();
      return status;
    }

    delay();
  }

  // write crc
  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
      (FLASH_SECTOR_3_ADDR + SIZE_OF_PROFILES), crc);

  HAL_FLASH_Lock();
  delay();

  return status;
}

static HAL_StatusTypeDef load_profiles(void)
{
  static profile_t _profile[NUM_OF_PROFILES];

  for (int i = 0; i < NUM_OF_PROFILES; i++)
  {
    for (int j = 0; j < SIZE_OF_PROFILE_IN_WORD; j++)
    {
      _profile[i].word[j] = *((__IO uint32_t*) (FLASH_SECTOR_3_ADDR
          + i * sizeof(profile_t) + j * 4));
    }
  }

  uint32_t stored_crc = *(__IO uint32_t*) (FLASH_SECTOR_3_ADDR + SIZE_OF_PROFILES);
  uint32_t calculated_crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) _profile,
      SIZE_OF_PROFILES_IN_WORD);

  if (stored_crc != calculated_crc)
  {
    return HAL_ERROR;
  }
  else
  {
    for (int i = 0; i < NUM_OF_PROFILES; i++)
    {
      profile[i] = _profile[i];
    }

    return HAL_OK;
  }
}



