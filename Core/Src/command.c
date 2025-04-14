/*
 * command.c
 *
 *  Created on: Apr 23, 2024
 *      Author: matianfu@gingerologist.com
 */
/* Includes ------------------------------------------------------------------*/

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "cmsis_os.h"
#include "command.h"
#include "main.h"
#include "profile.h"

/**
 * TODO https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
 * https://deepbluembedded.com/how-to-receive-uart-serial-data-with-stm32-dma-interrupt-polling/
 */

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static char long_buf[256];

EmbeddedCliConfig *cli_config = NULL;
EmbeddedCli *cli = NULL;

extern UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
static void cli_writeChar(EmbeddedCli *embeddedCli, char c);
static void CLI_CMD_List(EmbeddedCli *cli, char *args, void *context);
static void CLI_CMD_Define(EmbeddedCli *cli, char *args, void *context);

/* Private user code ---------------------------------------------------------*/
static void cli_writeChar(EmbeddedCli *embeddedCli, char c)
{
	uint8_t chr = c;
	HAL_UART_Transmit(&huart2, &chr, 1, HAL_MAX_DELAY);
}

static void CLI_CMD_List(EmbeddedCli *cli, char *args, void *context)
{
	for (int i = 0; i < 9; i++)
	{
		print_profile(i);
	}
}

CliCommandBinding cli_cmd_list_binding = {
		"list",
		"Print profile 1 to 9",
		false,
		NULL,
		CLI_CMD_List
};

static bool cfg_str_is_valid(const char* str)
{
	if (strlen(str) != 9) return false;
	for (int i = 0; i < 9; i++)
	{
		char c = str[i];
		if (c != '0' && c != '1' && c != '2')
		{
			return false;
		}
	}
	return true;
}

static bool parse_config(const char* str, uint32_t *config)
{
	if (!cfg_str_is_valid(str))
	{
		return false;
	}

	if (config != NULL)
	{
		*config = 0;
		for (int i = 0; i < 9; i++)
		{
			switch(str[i])
			{
			case '1':
				*config |= 0x01 << (i * 2);
				break;
			case '2':
				*config |= 0x02 << (i * 2);
				break;
			default:
				break;
			}
		}
	}

	return true;
}

// strictly 0 to 9, no leading zero, not larger than 3600 * 1000
static bool nat_is_valid(const char* str)
{
	size_t len = strlen(str);
	if (len == 0)
	{
		return false;
	}

	for (int i = 0; i < len; i++)
	{
		if (str[i] < '0' || str[i] > '9') return false;
	}

	if (str[0] == '0' && len > 1) return false;

	return true;
}

static void CLI_CMD_Define(EmbeddedCli *cli, char *args, void *context)
{
	uint8_t count = embeddedCliGetTokenCount(args);

	if (count != 6)
	{
		print_line("error: define command requires exact 6 arguments.");
		return;
	}

	const char *p = embeddedCliGetToken(args, 1);
	if (strlen(p) != 2 || p[0] < '1' || p[0] > '9' || (p[1] != 'a' && p[1] != 'b'))
	{
		print_line("error: first argument is invalid.");
		return;
	}

	int profile_index = p[0] - '1';
	int profile_phase = (p[1] == 'a') ? 0 : 1;
	uint32_t config[4];

	for (int j = 2; j < 6; j++)
	{
		p = embeddedCliGetToken(args, j);
		if (!parse_config(p, &config[j - 2]))
		{
			snprintf(long_buf, 256, "error: argument %d \"%s\" is invalid.", j, p);
			print_line(long_buf);
			return;
		}
	}

	const int max_str_len = strlen("3600");
	const uint32_t max_dur_value = 3600;

	p = embeddedCliGetToken(args, 6);
	if (!nat_is_valid(p))
	{
		print_line("error: duration is not a valid number.");
		return;
	}

	if (strlen(p) > max_str_len)
	{
		print_line("error: duration value is too large (max 3600)");
		return;
	}

	long int nat = strtol(p, NULL, 10);
	if (nat > max_dur_value)
	{
		print_line("error: duration value is too large (max 3600)");
		return;
	}

	uint32_t duration = (uint32_t)nat;

	if (profile_phase == 0)
	{
		set_profile(profile_index, config, &duration, NULL, NULL);
	}
	else
	{
		set_profile(profile_index, NULL, NULL, config, &duration);
	}

	print_profile(profile_index);
}

CliCommandBinding cli_cmd_define_binding = {
		"define",
		"Define a profile. Example:\r\n"
		"        > define 9a 222222222 111111111 222222222 111111111 3\r\n"
		"        > define 9b 111111111 222222222 111111111 222222222 3\r\n"
		"        > see more detail in manual."
		,
		true,
		NULL,
		CLI_CMD_Define
};

static void CLI_CMD_Blink(EmbeddedCli *cli, char *args, void *context)
{
	do_profile_blink();
}

CliCommandBinding cli_cmd_blink_binding = {
		"blink",
		"Blink all leds (for test purpose only).",
		false,
		NULL,
		CLI_CMD_Blink
};

static void CLI_CMD_Reboot(EmbeddedCli *cli, char *args, void *context)
{
	NVIC_SystemReset();
}

CliCommandBinding cli_cmd_reboot_binding = {
		"reboot",
		"Reboot the processor (for test purpose only).",
		false,
		NULL,
		CLI_CMD_Reboot
};

void StartUxTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  cli_config = embeddedCliDefaultConfig();
  cli = embeddedCliNew(cli_config);
  cli -> writeChar = cli_writeChar;

  embeddedCliAddBinding(cli, cli_cmd_blink_binding);
  embeddedCliAddBinding(cli, cli_cmd_list_binding);
  embeddedCliAddBinding(cli, cli_cmd_define_binding);
  embeddedCliAddBinding(cli, cli_cmd_reboot_binding);

  vTaskDelay(500);

  // generate the first prompt symbol
  embeddedCliReceiveChar(cli, 8);
  embeddedCliProcess(cli);

  /* Infinite loop */
  for(;;)
  {
	uint8_t c;
	HAL_StatusTypeDef status = HAL_UART_Receive(&huart2, &c, 1, 10);
	if (status == HAL_OK)
	{
	  embeddedCliReceiveChar(cli, c);
	  embeddedCliProcess(cli);
	}

	int key = detect_single_keydown();
	if (key >= 0)
	{
		do_profile_by_key(key);
	}

	// wait 10ms
	vTaskDelay(10);
  }
  /* USER CODE END 5 */
}



