/*
 * uart.c
 *
 *  Created on: May 2, 2026
 *      Author: matia
 */
#include <stdint.h>
#include <stdio.h>
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "main.h"

extern osMessageQId messageQueueHandle;
extern UART_HandleTypeDef huart6;

#define RX_BUF_SIZE 128
uint8_t rx_buffer[RX_BUF_SIZE];

typedef union {
  unsigned int word;
  uint8_t xbuf[4];
} message_t;

message_t message = {.word = 0xFFFFFFFF};
message_t loopback;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART6) {
    // handle data;
    if (Size == 4 && rx_buffer[0] == message.xbuf[0] &&
        rx_buffer[1] == message.xbuf[1] && rx_buffer[2] == message.xbuf[2] &&
        rx_buffer[3] == message.xbuf[3]) {
    } else {
      printf("uart rx, Size: %d bytes, first char (code): %u", Size,
             rx_buffer[0]);
    }

    // restart rx
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buffer, RX_BUF_SIZE);
  }
}

void UART_Send(uint32_t message) {
  xQueueSend(messageQueueHandle, &message, portMAX_DELAY);
}

void StartUartTask(void const *argument) {
  vTaskDelay(100);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buffer, RX_BUF_SIZE);
  for (;;) {
    xQueueReceive(messageQueueHandle, &message, portMAX_DELAY);
    HAL_UART_Transmit(&huart6, message.xbuf, 4, portMAX_DELAY);
    printf("uart tx %u (%08x)\r\n", message.word, message.word);

    vTaskDelay(200);
  }
  /* USER CODE END StartUartTask */
}
