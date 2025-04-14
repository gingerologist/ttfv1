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

#if 0

extern osMessageQId requestQueueHandle;

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct {
	char buf[10];
} pgcfg_strbuf_t;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
// see profile initializer below
#define STOP_PROFILE						profile[9]
#define DDBF_PROFILE						profile[10]
#define CURR_PROFILE						profile[11]
#define NEXT_PROFILE						profile[12]

/* Private variables ---------------------------------------------------------*/
static GPIO_TypeDef* s_port[4][9] =
{
	{ A1S_GPIO_Port, A2S_GPIO_Port, A3S_GPIO_Port, A4S_GPIO_Port, A5S_GPIO_Port, A6S_GPIO_Port, A7S_GPIO_Port, A8S_GPIO_Port, A9S_GPIO_Port },
	{ B1S_GPIO_Port, B2S_GPIO_Port, B3S_GPIO_Port, B4S_GPIO_Port, B5S_GPIO_Port, B6S_GPIO_Port, B7S_GPIO_Port, B8S_GPIO_Port, B9S_GPIO_Port },
	{ C1S_GPIO_Port, C2S_GPIO_Port, C3S_GPIO_Port, C4S_GPIO_Port, C5S_GPIO_Port, C6S_GPIO_Port, C7S_GPIO_Port, C8S_GPIO_Port, C9S_GPIO_Port },
	{ D1S_GPIO_Port, D2S_GPIO_Port, D3S_GPIO_Port, D4S_GPIO_Port, D5S_GPIO_Port, D6S_GPIO_Port, D7S_GPIO_Port, D8S_GPIO_Port, D9S_GPIO_Port },
};

static GPIO_TypeDef* c_port[4][9] =
{
	{ A1C_GPIO_Port, A2C_GPIO_Port, A3C_GPIO_Port, A4C_GPIO_Port, A5C_GPIO_Port, A6C_GPIO_Port, A7C_GPIO_Port, A8C_GPIO_Port, A9C_GPIO_Port },
	{ B1C_GPIO_Port, B2C_GPIO_Port, B3C_GPIO_Port, B4C_GPIO_Port, B5C_GPIO_Port, B6C_GPIO_Port, B7C_GPIO_Port, B8C_GPIO_Port, B9C_GPIO_Port },
	{ C1C_GPIO_Port, C2C_GPIO_Port, C3C_GPIO_Port, C4C_GPIO_Port, C5C_GPIO_Port, C6C_GPIO_Port, C7C_GPIO_Port, C8C_GPIO_Port, C9C_GPIO_Port },
	{ D1C_GPIO_Port, D2C_GPIO_Port, D3C_GPIO_Port, D4C_GPIO_Port, D5C_GPIO_Port, D6C_GPIO_Port, D7C_GPIO_Port, D8C_GPIO_Port, D9C_GPIO_Port },
};

static const uint16_t s_pin[4][9] =
{
	{ A1S_Pin, A2S_Pin, A3S_Pin, A4S_Pin, A5S_Pin, A6S_Pin, A7S_Pin, A8S_Pin, A9S_Pin },
	{ B1S_Pin, B2S_Pin, B3S_Pin, B4S_Pin, B5S_Pin, B6S_Pin, B7S_Pin, B8S_Pin, B9S_Pin },
	{ C1S_Pin, C2S_Pin, C3S_Pin, C4S_Pin, C5S_Pin, C6S_Pin, C7S_Pin, C8S_Pin, C9S_Pin },
	{ D1S_Pin, D2S_Pin, D3S_Pin, D4S_Pin, D5S_Pin, D6S_Pin, D7S_Pin, D8S_Pin, D9S_Pin },
};

static const uint16_t c_pin[4][9] =
{
	{ A1C_Pin, A2C_Pin, A3C_Pin, A4C_Pin, A5C_Pin, A6C_Pin, A7C_Pin, A8C_Pin, A9C_Pin },
	{ B1C_Pin, B2C_Pin, B3C_Pin, B4C_Pin, B5C_Pin, B6C_Pin, B7C_Pin, B8C_Pin, B9C_Pin },
	{ C1C_Pin, C2C_Pin, C3C_Pin, C4C_Pin, C5C_Pin, C6C_Pin, C7C_Pin, C8C_Pin, C9C_Pin },
	{ D1C_Pin, D2C_Pin, D3C_Pin, D4C_Pin, D5C_Pin, D6C_Pin, D7C_Pin, D8C_Pin, D9C_Pin },
};

static char long_buf[256] = {0};

/**
 * 0-8 		9 profiles
 * 9		all zero (used for stop)
 * 10		all deadbeef (used for blink)
 * 11		current
 * 12 		next
 */
static profile_t profile[13] = {
		{}, {}, {},
		{}, {}, {},
		{}, {}, {},
		{}, // profile[9]
		{	// profile[10]
			.pgcfg_a = { 0xdeadbeef, 0xdeadbeef, 0xdeadbeef, 0xdeadbeef },
			.pgcfg_b = { 0xdeadbeef, 0xdeadbeef, 0xdeadbeef, 0xdeadbeef },
			.duration_a_sec = 0xdeadbeef,
			.duration_b_sec = 0xdeadbeef
		},
		{}, {}};	// profile[11], profile[12]

extern CRC_HandleTypeDef hcrc;

/* Private function prototypes -----------------------------------------------*/
static void print_pad_group_config(uint32_t pg_cfg, pgcfg_strbuf_t *buf);
static HAL_StatusTypeDef save_profiles(void);
static void load_profiles(void);

/* Private user code ---------------------------------------------------------*/

static void print_pad_group_config(uint32_t pg_cfg, pgcfg_strbuf_t* buf)
{
	if (buf == NULL) return;

	for (int i = 0; i < 9; i++)
	{
		uint32_t bits = (pg_cfg >> (2 * i)) & 0x00000003;
		if (bits == 0)
		{
			buf->buf[i] = '0';
		}
		else if (bits == 1)
		{
			buf->buf[i] = '1';
		}
		else if (bits == 2)
		{
			buf->buf[i] = '2';
		}
		else
		{
			buf->buf[i] = '-';
		}
	}

	buf->buf[9] = '\0';
}

/* Public user code ---------------------------------------------------------*/
void print_profile(int i)
{
	static pgcfg_strbuf_t buf[4];

	for (int j = 0; j < 4; j++)
	{
		print_pad_group_config(profile[i].pgcfg_a[j], &buf[j]);
	}

	snprintf(long_buf,
			256,
			"Profile #%d phase a: %s %s %s %s, duration: %ld sec",
			i + 1,
			buf[0].buf,
			buf[1].buf,
			buf[2].buf,
			buf[3].buf,
			profile[i].duration_a_sec);
	print_line(long_buf);

	for (int j = 0; j < 4; j++)
	{
		print_pad_group_config(profile[i].pgcfg_b[j], &buf[j]);
	}

	snprintf(long_buf,
			256,
			"           phase b: %s %s %s %s, duration: %ld sec",
			buf[0].buf,
			buf[1].buf,
			buf[2].buf,
			buf[3].buf,
			profile[i].duration_b_sec);
	print_line(long_buf);

	print_line(NULL);
}

profile_t get_profile(int index)
{
	if (index > -1 && index < 9)
	{
		return profile[index];
	}

	profile_t profile = {0};
	return profile;
}

void set_profile(int index,
				uint32_t pgcfg_a[4],
				uint32_t *duration_a_sec,
				uint32_t pgcfg_b[4],
				uint32_t *duration_b_sec)
{
	if (pgcfg_a != NULL)
	{
		profile[index].pgcfg_a[0] = pgcfg_a[0];
		profile[index].pgcfg_a[1] = pgcfg_a[1];
		profile[index].pgcfg_a[2] = pgcfg_a[2];
		profile[index].pgcfg_a[3] = pgcfg_a[3];
	}

	if (duration_a_sec != NULL)
	{
		profile[index].duration_a_sec = *duration_a_sec;
	}

	if (pgcfg_b != NULL)
	{
		profile[index].pgcfg_b[0] = pgcfg_b[0];
		profile[index].pgcfg_b[1] = pgcfg_b[1];
		profile[index].pgcfg_b[2] = pgcfg_b[2];
		profile[index].pgcfg_b[3] = pgcfg_b[3];
	}

	if (duration_b_sec != NULL)
	{
		profile[index].duration_b_sec = *duration_b_sec;
	}

	save_profiles();
}

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

void do_profile_blink()
{
	xQueueSend(requestQueueHandle, &profile[10], 0);
}


void StartProfileTask(void const * argument)
{
	vTaskDelay(100);

	load_profiles();

entry_point:
	CURR_PROFILE = NEXT_PROFILE;

	if (CURR_PROFILE.pgcfg_a[0] == 0xdeadbeef)
	{
		goto profile_blink;
	}

	for (;;)
	{
		uint32_t dur;

		update_all_switches(CURR_PROFILE.pgcfg_a);
		dur = CURR_PROFILE.duration_a_sec * 1000;
		if (dur == 0) dur = portMAX_DELAY;

		if (pdTRUE == xQueueReceive(requestQueueHandle, &NEXT_PROFILE, dur))
		{
			goto entry_point;
		}

		update_all_switches(CURR_PROFILE.pgcfg_b);
		dur = CURR_PROFILE.duration_b_sec * 1000;
		if (dur == 0) dur = portMAX_DELAY;

		if (pdTRUE == xQueueReceive(requestQueueHandle, &NEXT_PROFILE, dur))
		{
			goto entry_point;
		}
	}

profile_blink:
	for (int port = 0; port < 4; port++)
	{
		for (int pin = 0; pin < 9; pin++)
		{
			update_switch(port, pin, GPIO_PIN_RESET, GPIO_PIN_RESET);
		}
	}

	for (int port = 0; port < 4; port++)
	{
		for (int pin = 0; pin < 9; pin++)
		{
			update_switch(port, pin, GPIO_PIN_RESET, GPIO_PIN_SET);
			// vTaskDelay(100);
			if (pdTRUE == xQueueReceive(requestQueueHandle, &NEXT_PROFILE, 100))
			{
				goto entry_point;
			}
		}
	}

	for (int port = 0; port < 4; port++)
	{
		for (int pin = 0; pin < 9; pin++)
		{
			update_switch(port, pin, GPIO_PIN_SET, GPIO_PIN_RESET);
			// vTaskDelay(100);
			if (pdTRUE == xQueueReceive(requestQueueHandle, &NEXT_PROFILE, 100))
			{
				goto entry_point;
			}
		}
	}

	for (int port = 0; port < 4; port++)
	{
		for (int pin = 0; pin < 9; pin++)
		{
			update_switch(port, pin, GPIO_PIN_RESET, GPIO_PIN_RESET);
			// vTaskDelay(100);
			if (pdTRUE == xQueueReceive(requestQueueHandle, &NEXT_PROFILE, 100))
			{
				goto entry_point;
			}
		}
	}

	NEXT_PROFILE = STOP_PROFILE;
	goto entry_point;
}

// page size 2KB, table 29 flash memory characteristics, in datasheet (stm32f103ze.pdf)
// stm32f103zEt6 where E stands for 512Kbytes of Flash memory
// flash start 	0x08000000
// flash size  	0x00080000
// page        	0x00000800
// lastpg start	0x0807f800

// FLASH_PAGE_SIZE				0x0800			// 2KB, already defined in stm32f1xx_hal_flash_ex.h
#define FLASH_SIZE				0x080000		// 512KB
#define FLASH_ADDR_BASE			0x08000000
#define LAST_PAGE_ADDR 			(FLASH_ADDR_BASE + FLASH_SIZE - FLASH_PAGE_SIZE)

static void delay(void)
{
	vTaskDelay(50);
}

static HAL_StatusTypeDef save_profiles(void)
{
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint64_t *dword;

	uint32_t calc = HAL_CRC_Calculate(&hcrc, (uint32_t *)profile, 90);

	// unlock flash
    HAL_FLASH_Unlock();

    // erase page
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks = FLASH_BANK_1; // FLASHEx_Banks, possibly only required for mass erasure
	EraseInitStruct.PageAddress = LAST_PAGE_ADDR;
	EraseInitStruct.NbPages = 1;

	uint32_t PageError;
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)		// Erase the Page Before a Write Operation
	{
		return HAL_ERROR;
	}
	delay();

	// write profile
	dword = (uint64_t *)profile;
	for (int i = 0; i < 45; i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (LAST_PAGE_ADDR + i * sizeof(uint64_t)), dword[i]);
		delay();
	}

	// write crc
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (LAST_PAGE_ADDR + 45 * sizeof(uint64_t)), (uint64_t)calc);
	delay();

	// unlock flash
	HAL_FLASH_Lock();
	return HAL_OK;
}

static void load_profiles(void)
{
	static profile_t prfl[9];

	for (int i = 0; i < 9; i++)
	{
		prfl[i].pgcfg_a[0] 		= *(__IO uint32_t *)(LAST_PAGE_ADDR + i * 40 + 0);
		prfl[i].pgcfg_a[1] 		= *(__IO uint32_t *)(LAST_PAGE_ADDR + i * 40 + 4);
		prfl[i].pgcfg_a[2] 		= *(__IO uint32_t *)(LAST_PAGE_ADDR + i * 40 + 8);
		prfl[i].pgcfg_a[3] 		= *(__IO uint32_t *)(LAST_PAGE_ADDR + i * 40 + 12);
		prfl[i].pgcfg_b[0] 		= *(__IO uint32_t *)(LAST_PAGE_ADDR + i * 40 + 16);
		prfl[i].pgcfg_b[1] 		= *(__IO uint32_t *)(LAST_PAGE_ADDR + i * 40 + 20);
		prfl[i].pgcfg_b[2] 		= *(__IO uint32_t *)(LAST_PAGE_ADDR + i * 40 + 24);
		prfl[i].pgcfg_b[3] 		= *(__IO uint32_t *)(LAST_PAGE_ADDR + i * 40 + 28);
		prfl[i].duration_a_sec	= *(__IO uint32_t *)(LAST_PAGE_ADDR + i * 40 + 32);
		prfl[i].duration_b_sec 	= *(__IO uint32_t *)(LAST_PAGE_ADDR + i * 40 + 36);
	}

	uint32_t crc = *(__IO uint32_t *)(LAST_PAGE_ADDR + 9 * 40);
	uint32_t calc = HAL_CRC_Calculate(&hcrc, (uint32_t *)prfl, 90);

	if (crc != calc)
	{
		print_line("error: bad crc when loading profiles.");
	}
	else
	{
		for (int i = 0; i < 9; i++)
		{
			profile[i] = prfl[i];
		}

		print_line("profiles loaded.");
	}
}

#endif

