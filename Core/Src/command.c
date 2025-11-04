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
#include "tca9555.h"
#include "edge_detect.h"

/**
 * SOMEDAY https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
 * https://deepbluembedded.com/how-to-receive-uart-serial-data-with-stm32-dma-interrupt-polling/
 */

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

const static phase_t zero_phase = {0};

EmbeddedCliConfig *cli_config = NULL;
EmbeddedCli *cli = NULL;

extern UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
static void cli_writeChar(EmbeddedCli *embeddedCli, char c);
static void CLI_CMD_List(EmbeddedCli *cli, char *args, void *context);
static void CLI_CMD_Define(EmbeddedCli *cli, char *args, void *context);

/* Private user code ---------------------------------------------------------*/
static void cli_writeChar(EmbeddedCli *embeddedCli, char c) {
  uint8_t chr = c;
  HAL_UART_Transmit(&huart2, &chr, 1, HAL_MAX_DELAY);
}

static void CLI_CMD_List(EmbeddedCli *cli, char *args, void *context) {
  for (int i = 0; i < 16; i++) {
    print_profile(i);
  }
}

CliCommandBinding cli_cmd_list_binding = {"list", "Print profile 0 to 15",
                                          false, NULL, CLI_CMD_List};

/**
 * @brief  Parses a string containing a number from "0" to "15" and converts it
 * to an integer index.
 * @param  str: Pointer to the input string to be parsed
 * @param  index: Pointer to store the resulting integer index (0-15)
 * @retval true if parsing was successful (input was "0" to "15"), false
 * otherwise
 */
static bool parse_profile_index(const char *str, int *index) {
  size_t len = strlen(str);
  if (len == 1 && str[0] >= '0' && str[0] <= '9') {
    *index = str[0] - '0'; // Convert '0'-'9' to 0-9
    return true;
  }
  if (len == 2 && str[0] == '1' && str[1] >= '0' && str[1] <= '5') {
    *index = 10 + (str[1] - '0'); // Convert "10"-"15" to 10-15
    return true;
  }
  return false;
}

/**
 * @brief  Parses a single character string to determine the phase index.
 * @param  str: Pointer to the input string to be parsed
 * @param  phase: Pointer to store the resulting phase index (0 for 'a', 1 for
 * 'b')
 * @retval true if parsing was successful (input was 'a' or 'b'), false
 * otherwise
 */
static bool parse_phase_index(const char *str, int *phase) {
  size_t len = strlen(str);

  if (len > 1) {
    return false;
  }

  if (str[0] == 'a') {
    *phase = 0;
    return true;
  }

  if (str[0] == 'b') {
    *phase = 1;
    return true;
  }

  return false;
}

/**
 * @brief  Parses a string containing pad configuration data and populates the
 * padscfg structure.
 * @param  str: Pointer to the input string containing pad configuration values
 *              Format must be exactly 53 characters long with specific format:
 *              - Values must be '0', '1', or '2'
 *              - Commas at positions divisible by 6 (except when divisible by
 * 18)
 *              - Semicolons at positions divisible by 18
 * @param  padscfg: Pointer to allpads_t structure to store the parsed
 * configuration values
 * @retval true if parsing was successful (valid format and values), false
 * otherwise
 *
 * @note   The function maps specific string positions to corresponding pad
 * configuration fields in a row-based structure for different phases (a, b, c)
 * and positions (top, left, mid, right, bottom).
 */
static bool parse_phase_padscfg(const char *str, allpads_t *padscfg) {
  size_t len = strlen(str);
  if (len != 53) {
    return false;
  }

  padscfg->row[0].word = 0;
  padscfg->row[1].word = 0;
  padscfg->row[2].word = 0;

  for (int i = 0; i < 53; i++) {
    if ((i + 1) % 6 == 0) {
      if ((i + 1) % 18 == 0) {
        if (str[i] != ';') {
          return false;
        }
      } else {
        if (str[i] != ',') {
          return false;
        }
      }
    } else if (str[i] != '0' && str[i] != '1' && str[i] != '2') {
      return false;
    }

    switch (i) {
    case 0:
      padscfg->row[0].a_top = str[i] - '0';
      break;
    case 1:
      padscfg->row[0].a_lft = str[i] - '0';
      break;
    case 2:
      padscfg->row[0].a_mid = str[i] - '0';
      break;
    case 3:
      padscfg->row[0].a_rgt = str[i] - '0';
      break;
    case 4:
      padscfg->row[0].a_bot = str[i] - '0';
      break;

    case 6:
      padscfg->row[0].b_top = str[i] - '0';
      break;
    case 7:
      padscfg->row[0].b_lft = str[i] - '0';
      break;
    case 8:
      padscfg->row[0].b_mid = str[i] - '0';
      break;
    case 9:
      padscfg->row[0].b_rgt = str[i] - '0';
      break;
    case 10:
      padscfg->row[0].b_bot = str[i] - '0';
      break;

    case 12:
      padscfg->row[0].c_top = str[i] - '0';
      break;
    case 13:
      padscfg->row[0].c_lft = str[i] - '0';
      break;
    case 14:
      padscfg->row[0].c_mid = str[i] - '0';
      break;
    case 15:
      padscfg->row[0].c_rgt = str[i] - '0';
      break;
    case 16:
      padscfg->row[0].c_bot = str[i] - '0';
      break;

    case 18:
      padscfg->row[1].a_top = str[i] - '0';
      break;
    case 19:
      padscfg->row[1].a_lft = str[i] - '0';
      break;
    case 20:
      padscfg->row[1].a_mid = str[i] - '0';
      break;
    case 21:
      padscfg->row[1].a_rgt = str[i] - '0';
      break;
    case 22:
      padscfg->row[1].a_bot = str[i] - '0';
      break;

    case 24:
      padscfg->row[1].b_top = str[i] - '0';
      break;
    case 25:
      padscfg->row[1].b_lft = str[i] - '0';
      break;
    case 26:
      padscfg->row[1].b_mid = str[i] - '0';
      break;
    case 27:
      padscfg->row[1].b_rgt = str[i] - '0';
      break;
    case 28:
      padscfg->row[1].b_bot = str[i] - '0';
      break;

    case 30:
      padscfg->row[1].c_top = str[i] - '0';
      break;
    case 31:
      padscfg->row[1].c_lft = str[i] - '0';
      break;
    case 32:
      padscfg->row[1].c_mid = str[i] - '0';
      break;
    case 33:
      padscfg->row[1].c_rgt = str[i] - '0';
      break;
    case 34:
      padscfg->row[1].c_bot = str[i] - '0';
      break;

    case 36:
      padscfg->row[2].a_top = str[i] - '0';
      break;
    case 37:
      padscfg->row[2].a_lft = str[i] - '0';
      break;
    case 38:
      padscfg->row[2].a_mid = str[i] - '0';
      break;
    case 39:
      padscfg->row[2].a_rgt = str[i] - '0';
      break;
    case 40:
      padscfg->row[2].a_bot = str[i] - '0';
      break;

    case 42:
      padscfg->row[2].b_top = str[i] - '0';
      break;
    case 43:
      padscfg->row[2].b_lft = str[i] - '0';
      break;
    case 44:
      padscfg->row[2].b_mid = str[i] - '0';
      break;
    case 45:
      padscfg->row[2].b_rgt = str[i] - '0';
      break;
    case 46:
      padscfg->row[2].b_bot = str[i] - '0';
      break;

    case 48:
      padscfg->row[2].c_top = str[i] - '0';
      break;
    case 49:
      padscfg->row[2].c_lft = str[i] - '0';
      break;
    case 50:
      padscfg->row[2].c_mid = str[i] - '0';
      break;
    case 51:
      padscfg->row[2].c_rgt = str[i] - '0';
      break;
    case 52:
      padscfg->row[2].c_bot = str[i] - '0';
      break;

    default:
      break;
    }
  }

  return true;
}

/**
 * @brief  Parses a string to extract a phase duration value.
 * @param  p: Pointer to the input string to be parsed
 * @param  duration: Pointer to store the resulting duration value
 * @retval true if parsing was successful, false otherwise
 *
 * @note   The function validates that:
 *         - Input pointers are not NULL
 *         - Input string contains only a valid integer
 *         - The value is within the allowed range (0-3600)
 */
static bool parse_phase_duration(const char *p, int *duration) {
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

  *duration = (int)value;
  return true;
}

/**
 * @brief  Parses a string to extract a phase level value.
 * @param  p: Pointer to the input string to be parsed
 * @param  level: Pointer to store the resulting level value
 * @retval true if parsing was successful, false otherwise
 *
 * @note   The function validates that:
 *         - Input pointers are not NULL
 *         - Input string contains only a valid integer
 *         - The value is within the allowed range (0-100)
 */
static bool parse_phase_level(const char *p, int *level) {
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

  *level = (int)value;
  return true;
}

static bool parse_phase_freq(const char *p, uint32_t *freq) {
  char *endptr;
  long value;

  // Check for null pointer
  if (p == NULL || freq == NULL)
    return false;

  // Convert string to long integer
  value = strtol(p, &endptr, 10);

  // Check that the entire string was consumed (valid number only)
  if (*endptr != '\0')
    return false;

  *freq = (int)value;
  return true;
}

/*
 * example:
 *  define 1 a 11111,11111,11111;22222,22222,22222;00000,00000,00000 33 100
 *
 *  where 1     0-15 [1-based index]
 *        a     a or b
 *        11111,11111,11111;22222,22222,22222;00000,00000,00000
 *        33    duration in seconds
 *        100   voltage level
 */
static void CLI_CMD_Define(EmbeddedCli *cli, char *args, void *context) {
  const char *p;
  int profile_index;
  int phase_index;
  phase_t phase;

  uint16_t count = embeddedCliGetTokenCount(args);

  if (count != 6) {
    printf("error: define command requires exact 5 arguments.\r\n");
    return;
  }

  p = embeddedCliGetToken(args, 1);
  if (!parse_profile_index(p, &profile_index)) {
    printf("error: invalid profile index.\r\n");
    return;
  }

  p = embeddedCliGetToken(args, 2);
  if (!parse_phase_index(p, &phase_index)) {
    printf("error: invalid profile phase.\r\n");
    return;
  }

  p = embeddedCliGetToken(args, 3);
  if (!parse_phase_padscfg(p, &phase.pads)) {
    printf("error: invalid pads config.\r\n");
    return;
  }

  p = embeddedCliGetToken(args, 4);
  if (!parse_phase_duration(p, &phase.duration)) {
    printf("error: invalid phase duration.\r\n");
    return;
  }

  p = embeddedCliGetToken(args, 5);
  if (!parse_phase_level(p, &phase.level)) {
    printf("error: invalid phase level.\r\n");
    return;
  }

  p = embeddedCliGetToken(args, 6);
  if (!parse_phase_freq(p, &phase.freq)) {
    printf("error: invalid phase frequency.\r\n");
    return;
  }

  // Check range
  if (phase.freq < 50000 || phase.freq > 500000) {
    printf("error: invalid frequency range (50000 to 500000).\r\n");
    return;
  }

  set_profile_phase(profile_index, phase_index, &phase);

  print_profile(profile_index);
}

CliCommandBinding cli_cmd_define_binding = {
    "define",
    "Define a profile. Example:\r\n"
    "        # define profile 0, phase a, with 30 seconds, 100% voltage, and "
    "5000 Hz\r\n\r\n"
    "        > define 0 a "
    "11111,11111,11111;22222,22222,22222;00000,00000,00000 30 100 5000\r\n\r\n"
    "        # define profile 15, phase b, with 3600 seconds, 10% voltage, and "
    "500 KHz\r\n\r\n"
    "        > define 15 b "
    "11111,11111,11111;22222,22222,22222;00000,00000,00000 3600 10 "
    "500000\r\n\r\n",
    true, NULL, CLI_CMD_Define};

// static void CLI_CMD_Save(EmbeddedCli *cli, char *args, void *context) {}

static void CLI_CMD_Clear(EmbeddedCli *cli, char *args, void *context) {
  // printf("\033[2J\033[H"); This does not work
  // printf("\x1B[2J\x1B[H"); This does not work either
  HAL_UART_Transmit(&huart2, (uint8_t *)"\x1B[2J\x1B[H", 7,
                    100); // this works :)
}

CliCommandBinding cli_cmd_clear_binding = {"clear", "Clear the screen.", false,
                                           NULL, CLI_CMD_Clear};

static void CLI_CMD_Reboot(EmbeddedCli *cli, char *args, void *context) {
  NVIC_SystemReset();
}

CliCommandBinding cli_cmd_reboot_binding = {"reboot", "Reboot the processor.",
                                            false, NULL, CLI_CMD_Reboot};

/*
 * Active Low (There is an ON label on switch)
 */
static void CLI_CMD_Dip(EmbeddedCli *cli, char *args, void *context) {
  printf(
      "SW1: %s, SW2: %s, SW3: %s, SW4: %s, SW5: %s%s",
      (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_SET) ? "off" : "on",
      (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_SET) ? "off" : "on",
      (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == GPIO_PIN_SET) ? "off" : "on",
      (HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin) == GPIO_PIN_SET) ? "off" : "on",
      (HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin) == GPIO_PIN_SET) ? "off" : "on",
      "\r\n");
}

CliCommandBinding cli_cmd_dip_binding = {
    "dip", "Show on board DIP switch states.", false, NULL, CLI_CMD_Dip};

static void CLI_CMD_LED(EmbeddedCli *cli, char *args, void *context) {
  HAL_GPIO_TogglePin(LEDC_GPIO_Port, LEDC_Pin);
  vTaskDelay(500);
  HAL_GPIO_TogglePin(LEDC_GPIO_Port, LEDC_Pin);
  vTaskDelay(500);
  HAL_GPIO_TogglePin(LEDC_GPIO_Port, LEDC_Pin);
  vTaskDelay(500);
  HAL_GPIO_TogglePin(LEDC_GPIO_Port, LEDC_Pin);
  vTaskDelay(500);
  HAL_GPIO_TogglePin(LEDC_GPIO_Port, LEDC_Pin);
  vTaskDelay(500);
  HAL_GPIO_TogglePin(LEDC_GPIO_Port, LEDC_Pin);
  vTaskDelay(500);
}

CliCommandBinding cli_cmd_led_binding = {
    "led", "blink led (for testing purposes).", false, NULL, CLI_CMD_LED};

static void CLI_CMD_LL(EmbeddedCli *cli, char *args, void *context) {
  printf("DDBF profile:\r\n");
  print_profile(DDBF_PROFILE_INDEX);
  printf("CURR profile:\r\n");
  print_profile(CURR_PROFILE_INDEX);
  printf("NEXT profile:\r\n");
  print_profile(NEXT_PROFILE_INDEX);
}

CliCommandBinding cli_cmd_ll_binding = {
    "ll", "Print internal profiles (for testing purposes only)", false, NULL,
    CLI_CMD_LL};

static void CLI_CMD_TGroup(EmbeddedCli *cli, char *args, void *context) {
  static phase_t phase = {.level = 10};
  const char *p;
  char padstr[54] = "00000,00000,00000;00000,00000,00000;00000,00000,00000";
  int pos;

  uint16_t count = embeddedCliGetTokenCount(args);

  if (count != 2) {
    printf("error: t1 command requires exact 2 arguments.\r\n");
    return;
  }

  p = embeddedCliGetToken(args, 1);
  if (strlen(p) != 1 || *p < '0' || *p > '8') {
    printf("error: the first argument must be 0-8\r\n");
    return;
  }

  pos = *p - '0';

  p = embeddedCliGetToken(args, 2);
  if (strlen(p) != 5) {
    printf("error: the 2nd argument must contain exact 5 symbols.\r\n");
    return;
  }

  for (int i = 0; i < 5; i++) {
    if (p[i] != '0' && p[i] != '1' && p[i] != '2') {
      printf("error: all symbols in the 2nd argument must be 0, 1 or 2.\r\n");
      return;
    }

    padstr[pos * 6 + i] = p[i];
  }

  printf("padstr: %s\r\n", padstr);

  if (!parse_phase_padscfg(padstr, &phase.pads)) {
    printf("error: unexpected error.\r\n");
    return;
  }

  set_profile_phase(DDBF_PROFILE_INDEX, 0, &phase);
  set_profile_phase(DDBF_PROFILE_INDEX, 1, &zero_phase);

  do_profile(DDBF_PROFILE_INDEX);
}

#if 0
/*
 * t1 0 00000
 */
static void CLI_CMD_T1(EmbeddedCli *cli, char *args, void *context)
{
  static phase_t phase;
  rowpads_t *row;
  const char *pos;
  const char *cfg;

  uint16_t count = embeddedCliGetTokenCount(args);

  if (count != 2)
  {
    printf("error: t1 command requires exact 2 arguments.\r\n");
    return;
  }

  pos = embeddedCliGetToken(args, 1);
  if (strlen(pos) != 1 || *pos < '0' || *pos > '8')
  {
    printf("error: the first argument must be 0-8\r\n");
  }

  cfg = embeddedCliGetToken(args, 2);
  if (strlen(cfg) != 5 || cfg[0] < '0' || cfg[0] > '2' || cfg[1] < '0'
      || cfg[1] > '2' || cfg[2] < '0' || cfg[2] > '2' || cfg[3] < '0'
      || cfg[3] > '2' || cfg[4] < '0' || cfg[4] > '2')
  {
    printf(
        "error: the second argument must be nnnnn where n is 0, 1, or 2\r\n");
  }

  memset(&phase, 0, sizeof(phase));
  set_profile_phase(DDBF_PROFILE_INDEX, 1, &phase);

  phase.level = 20;

  row = &phase.pads.row[(*pos - '0') / 3];

  switch ((*pos - '0') % 3)
  {
    case 0:
      row->a_top = cfg[0] - '0';
      row->a_lft = cfg[1] - '0';
      row->a_mid = cfg[2] - '0';
      row->a_rgt = cfg[3] - '0';
      row->a_bot = cfg[4] - '0';
      break;
    case 1:
      row->b_top = cfg[0] - '0';
      row->b_lft = cfg[1] - '0';
      row->b_mid = cfg[2] - '0';
      row->b_rgt = cfg[3] - '0';
      row->b_bot = cfg[4] - '0';
      break;
    case 2:
      row->c_top = cfg[0] - '0';
      row->c_lft = cfg[1] - '0';
      row->c_mid = cfg[2] - '0';
      row->c_rgt = cfg[3] - '0';
      row->c_bot = cfg[4] - '0';
      break;
    default:
      break;
  }

  set_profile_phase(DDBF_PROFILE_INDEX, 0, &phase);

  do_profile(DDBF_PROFILE_INDEX);
}
#endif

CliCommandBinding cli_cmd_tg_binding = {
    "tg", "Direct config a single group of pads, for testing purposes only.",
    true, NULL, CLI_CMD_TGroup};

#ifdef TCA9555
static void CLI_CMD_Port(EmbeddedCli *cli, char *args, void *context) {
  TCA9555_Dump();
}

CliCommandBinding cli_cmd_port_binding = {
    "port", "Print output value of all tca9555 ports.", false, NULL,
    CLI_CMD_Port};
#endif

void StartUxTask(void const *argument) {
  static SW_HandleTypeDef sw5_handle = {
      .status = SW_RELEASED,
      .count = 0,
      .max_count = 3 // Default debounce
  };

  /* USER CODE BEGIN 5 */
  cli_config = embeddedCliDefaultConfig();
  cli = embeddedCliNew(cli_config);
  cli->writeChar = cli_writeChar;

  embeddedCliAddBinding(cli, cli_cmd_list_binding);

  embeddedCliAddBinding(cli, cli_cmd_define_binding);
  embeddedCliAddBinding(cli, cli_cmd_clear_binding);
  embeddedCliAddBinding(cli, cli_cmd_reboot_binding);
  embeddedCliAddBinding(cli, cli_cmd_dip_binding);

  // embeddedCliAddBinding(cli, cli_cmd_led_binding);

  embeddedCliAddBinding(cli, cli_cmd_ll_binding);
  embeddedCliAddBinding(cli, cli_cmd_tg_binding);

#ifdef TCA9555
  embeddedCliAddBinding(cli, cli_cmd_port_binding);
#endif

  vTaskDelay(100);

  // generate the first prompt symbol
  embeddedCliReceiveChar(cli, 8);
  embeddedCliProcess(cli);

  /* Infinite loop */
  for (;;) {
    uint8_t c;
    HAL_StatusTypeDef status = HAL_UART_Receive(&huart2, &c, 1, 10);
    if (status == HAL_OK) {
      embeddedCliReceiveChar(cli, c);
      embeddedCliProcess(cli);
    }

    // All switches are actively low. That is, when switch to ON, a falling edge
    // is detected.
    int edge =
        GPIO_EdgeDetect(&sw5_handle, HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin));
    if (edge < 0) {
      unsigned int bits = 0;

      if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin)) {
        bits |= 1;
      }

      if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin)) {
        bits |= 2;
      }

      if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin)) {
        bits |= 4;
      }

      if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin)) {
        bits |= 8;
      }

      printf("SW5 ON, do profile %d\r\n", bits);

      do_profile(bits);
      HAL_GPIO_WritePin(LEDC_GPIO_Port, LEDC_Pin, GPIO_PIN_SET);
    }

    if (edge > 0) {
      printf("SW5 OFF, do STOP profile\r\n");

      do_profile(STOP_PROFILE_INDEX);
      HAL_GPIO_WritePin(LEDC_GPIO_Port, LEDC_Pin, GPIO_PIN_RESET);
    }

    // wait 10ms
    vTaskDelay(10);
  }
  /* USER CODE END 5 */
}
