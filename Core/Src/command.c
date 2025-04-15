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
 * SOMEDAY https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
 * https://deepbluembedded.com/how-to-receive-uart-serial-data-with-stm32-dma-interrupt-polling/
 */

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
// static char long_buf[256];

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

CliCommandBinding cli_cmd_list_binding =
{ "list",
  "Print profile 1 to 9",
  false,
  NULL,
  CLI_CMD_List };

#if 0
static bool parse_config(const char *str, uint32_t *config)
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
      switch (str[i])
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
#endif

/*
 * Parses a string to get a zero-based index (0-15)
 * Valid inputs: "1" through "16"
 * Returns: true if valid, false otherwise
 * *index is zero-based (0-15)
 */
static bool parse_profile_index(const char *str, int *index)
{
  size_t len = strlen(str);

  if (len == 1 && str[0] >= '1' && str[0] <= '9')
  {
    *index = str[0] - '1';  // Convert '1'-'9' to 0-8
    return true;
  }

  if (len == 2 && str[0] == '1' && str[1] >= '0' && str[1] <= '6')
  {
    *index = 9 + (str[1] - '0');  // Convert "10"-"16" to 9-15
    return true;
  }

  return false;
}

static bool parse_phase_index(const char *str, int *phase)
{
  size_t len = strlen(str);

  if (len > 1)
  {
    return false;
  }

  if (str[0] == 'a')
  {
    *phase = 0;
    return true;
  }

  if (str[0] == 'b')
  {
    *phase = 1;
    return true;
  }

  return false;
}

/*
 * 11111,11111,11111;22222,22222,22222;00000,00000,00000
 */
static bool parse_phase_padscfg(const char *str, allpads_t* padscfg)
{
  size_t len = strlen(str);
  if (len != 53)
  {
    return false;
  }

  for (int i = 0; i < 53; i++)
  {
    if ((i + 1) % 6 == 0)
    {
      if ((i + 1) % 18 == 0)
      {
        if (str[i] != ';')
        {
          return false;
        }
      }
      else
      {
        if (str[i] != ',')
        {
          return false;
        }
      }
    }

    if (str[i] != '0' && str[i] != '1' && str[i] != '2')
    {
      return false;
    }

    switch (i)
    {
      case 0: padscfg->row[0].a_top = str[i] - '0'; break;
      case 1: padscfg->row[0].a_lft = str[i] - '0'; break;
      case 2: padscfg->row[0].a_mid = str[i] - '0'; break;
      case 3: padscfg->row[0].a_rgt = str[i] - '0'; break;
      case 4: padscfg->row[0].a_bot = str[i] - '0'; break;

      case 6: padscfg->row[0].b_top = str[i] - '0'; break;
      case 7: padscfg->row[0].b_lft = str[i] - '0'; break;
      case 8: padscfg->row[0].b_mid = str[i] - '0'; break;
      case 9: padscfg->row[0].b_rgt = str[i] - '0'; break;
      case 10: padscfg->row[0].b_bot = str[i] - '0'; break;

      case 12: padscfg->row[1].c_top = str[i] - '0'; break;
      case 13: padscfg->row[1].c_lft = str[i] - '0'; break;
      case 14: padscfg->row[1].c_mid = str[i] - '0'; break;
      case 15: padscfg->row[1].c_rgt = str[i] - '0'; break;
      case 16: padscfg->row[1].c_bot = str[i] - '0'; break;

      case 18: padscfg->row[1].a_top = str[i] - '0'; break;
      case 19: padscfg->row[1].a_lft = str[i] - '0'; break;
      case 20: padscfg->row[1].a_mid = str[i] - '0'; break;
      case 21: padscfg->row[1].a_rgt = str[i] - '0'; break;
      case 22: padscfg->row[1].a_bot = str[i] - '0'; break;

      case 24: padscfg->row[1].b_top = str[i] - '0'; break;
      case 25: padscfg->row[1].b_lft = str[i] - '0'; break;
      case 26: padscfg->row[1].b_mid = str[i] - '0'; break;
      case 27: padscfg->row[1].b_rgt = str[i] - '0'; break;
      case 28: padscfg->row[1].b_bot = str[i] - '0'; break;

      case 30: padscfg->row[2].c_top = str[i] - '0'; break;
      case 31: padscfg->row[2].c_lft = str[i] - '0'; break;
      case 32: padscfg->row[2].c_mid = str[i] - '0'; break;
      case 33: padscfg->row[2].c_rgt = str[i] - '0'; break;
      case 34: padscfg->row[2].c_bot = str[i] - '0'; break;

      case 36: padscfg->row[2].a_top = str[i] - '0'; break;
      case 37: padscfg->row[2].a_lft = str[i] - '0'; break;
      case 38: padscfg->row[2].a_mid = str[i] - '0'; break;
      case 39: padscfg->row[2].a_rgt = str[i] - '0'; break;
      case 40: padscfg->row[2].a_bot = str[i] - '0'; break;

      case 42: padscfg->row[2].b_top = str[i] - '0'; break;
      case 43: padscfg->row[2].b_lft = str[i] - '0'; break;
      case 44: padscfg->row[2].b_mid = str[i] - '0'; break;
      case 45: padscfg->row[2].b_rgt = str[i] - '0'; break;
      case 46: padscfg->row[2].b_bot = str[i] - '0'; break;

      case 48: padscfg->row[2].c_top = str[i] - '0'; break;
      case 49: padscfg->row[2].c_lft = str[i] - '0'; break;
      case 50: padscfg->row[2].c_mid = str[i] - '0'; break;
      case 51: padscfg->row[2].c_rgt = str[i] - '0'; break;
      case 52: padscfg->row[2].c_bot = str[i] - '0'; break;

      default:
        break;
    }
  }

  return true;
}

static bool parse_phase_duration(const char *p, int *duration)
{
  char *endptr;
  long value;

  // Check for null pointer
  if (p == NULL || duration == NULL)
    return false;

  // Convert string to long integer
  value = strtol(p, &endptr, 10);

  // Check that the entire string was consumed (valid number only)
  if (*endptr != '\0')
    return false;

  // Check range
  if (value < 0 || value > 3600)
    return false;

  *duration = (int) value;
  return true;
}

static bool parse_phase_level(const char *p, int *level)
{
  char *endptr;
  long value;

  // Check for null pointer
  if (p == NULL || level == NULL)
    return false;

  // Convert string to long integer
  value = strtol(p, &endptr, 10);

  // Check that the entire string was consumed (valid number only)
  if (*endptr != '\0')
    return false;

  // Check range
  if (value < 0 || value > 100)
    return false;

  *level = (int) value;
  return true;
}

/*
 * example:
 *  define 1 a 11111,11111,11111;22222,22222,22222;00000,00000,00000 33 100
 *
 *  where 1 1-16 [1-based index]
 *        a a or b
 *
 *        33
 *        100 is level
 */
static void CLI_CMD_Define(EmbeddedCli *cli, char *args, void *context)
{
  const char *p;
  int profile_index;
  int phase_index;
  phase_t phase;

  uint8_t count = embeddedCliGetTokenCount(args);

  if (count != 5)
  {
    // print_line();
    printf("error: define command requires exact 4 arguments.\r\n");
    return;
  }

  p = embeddedCliGetToken(args, 1);
  if (!parse_profile_index(p, &profile_index))
  {
    printf("error: invalid profile index.\r\n");
    return;
  }

  p = embeddedCliGetToken(args, 2);
  if (!parse_phase_index(p, &phase_index))
  {
    printf("error: invalid profile phase.\r\n");
    return;
  }

  p = embeddedCliGetToken(args, 3);
  if (!parse_phase_padscfg(p, &phase.pads))
  {
    printf("error: invalid pads config.\r\n");
    return;
  }

  p = embeddedCliGetToken(args, 4);
  if (!parse_phase_duration(p, &phase.duration))
  {
    printf("error: invalid phase duration.\r\n");
    return;
  }

  p = embeddedCliGetToken(args, 5);
  if (!parse_phase_level(p, &phase.level))
  {
    printf("error: invalid phase level.\r\n");
    return;
  }

  set_profile_phase(profile_index, phase_index, &phase);

  print_profile(profile_index);
}

CliCommandBinding cli_cmd_define_binding =
{ "define",
  "Define a profile. Example:\r\n"
  "        > define 9a 222222222 111111111 222222222 111111111 3\r\n"
  "        > define 9b 111111111 222222222 111111111 222222222 3\r\n"
  "        > see more detail in manual.",
  true,
  NULL,
  CLI_CMD_Define };

static void CLI_CMD_Blink(EmbeddedCli *cli, char *args, void *context)
{
  do_profile_blink();
}

CliCommandBinding cli_cmd_blink_binding =
{ "blink",
  "Blink all leds (for test purpose only).",
  false,
  NULL,
  CLI_CMD_Blink };

static void CLI_CMD_Reboot(EmbeddedCli *cli, char *args, void *context)
{
  NVIC_SystemReset();
}

CliCommandBinding cli_cmd_reboot_binding =
{ "reboot",
  "Reboot the processor (for test purpose only).",
  false,
  NULL,
  CLI_CMD_Reboot };

void StartUxTask(void const *argument)
{
  /* USER CODE BEGIN 5 */
  cli_config = embeddedCliDefaultConfig();
  cli = embeddedCliNew(cli_config);
  cli->writeChar = cli_writeChar;

  embeddedCliAddBinding(cli, cli_cmd_blink_binding);
  embeddedCliAddBinding(cli, cli_cmd_list_binding);
  embeddedCliAddBinding(cli, cli_cmd_define_binding);
  embeddedCliAddBinding(cli, cli_cmd_reboot_binding);

  vTaskDelay(500);

  // generate the first prompt symbol
  embeddedCliReceiveChar(cli, 8);
  embeddedCliProcess(cli);

  /* Infinite loop */
  for (;;)
  {
    uint8_t c;
    HAL_StatusTypeDef status = HAL_UART_Receive(&huart2, &c, 1, 10);
    if (status == HAL_OK)
    {
      embeddedCliReceiveChar(cli, c);
      embeddedCliProcess(cli);
    }

#if 0
    int key = detect_single_keydown();
    if (key >= 0)
    {
      do_profile_by_key(key);
    }
#endif
    // wait 10ms
    vTaskDelay(10);
  }
  /* USER CODE END 5 */
}

