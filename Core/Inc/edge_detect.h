/*
 * edge_detect.h
 *
 *  Created on: Apr 17, 2025
 *      Author: matia
 */

#ifndef INC_EDGE_DETECT_H_
#define INC_EDGE_DETECT_H_

#include <stdint.h>

#include "stm32f4xx_hal.h"

//typedef enum {
//    SW_RELEASED,
//    SW_DEBOUNCING_DOWN,
//    SW_PRESSED,
//    SW_DEBOUNCING_UP
//} SW_Status;
//
//
//
//typedef struct {
//    SW_Status status;
//    uint8_t count;
//    uint8_t edge;        // 0 = no edge, 1 = down edge, 2 = up edge
//    uint8_t max_count;   // Configurable debounce threshold
//} SW_HandleTypeDef;

typedef enum {
    SW_RELEASED,
    SW_DEBOUNCING_DOWN,
    SW_PRESSED,
    SW_DEBOUNCING_UP
} SW_Status;

typedef struct {
    SW_Status status;
    uint8_t count;
    uint8_t max_count;   // Configurable debounce threshold
} SW_HandleTypeDef;

int GPIO_EdgeDetect(SW_HandleTypeDef* handle, GPIO_PinState current_state);

#endif /* INC_EDGE_DETECT_H_ */
