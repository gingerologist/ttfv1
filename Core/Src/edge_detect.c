#include "edge_detect.h"

/* Returns:
 *  1 for rising edge (RESET -> SET)
 * -1 for falling edge (SET -> RESET)
 *  0 for no edge detected
 */
int GPIO_EdgeDetect(SW_HandleTypeDef *handle, GPIO_PinState current_state) {
  int return_edge = 0;

  switch (handle->status) {
  case SW_RELEASED:
    if (current_state == GPIO_PIN_RESET) // Potential falling edge
    {
      handle->status = SW_DEBOUNCING_DOWN;
      handle->count = 1;
    }
    break;

  case SW_DEBOUNCING_DOWN:
    if (current_state == GPIO_PIN_RESET) {
      handle->count++;
      if (handle->count > handle->max_count) // Configurable debounce threshold
      {
        handle->status = SW_PRESSED;
        handle->count = 0;
        return_edge = -1; // Falling edge reported here after debounce
      }
    } else {
      handle->status = SW_RELEASED;
      handle->count = 0;
    }
    break;

  case SW_PRESSED:
    if (current_state == GPIO_PIN_SET) // Rising edge start
    {
      handle->status = SW_DEBOUNCING_UP;
      handle->count = 1;
    }
    break;

  case SW_DEBOUNCING_UP:
    if (current_state == GPIO_PIN_SET) {
      handle->count++;
      if (handle->count > handle->max_count) // Configurable debounce threshold
      {
        handle->status = SW_RELEASED;
        handle->count = 0;
        return_edge = 1; // Rising edge reported here after debounce
      }
    } else {
      handle->status = SW_PRESSED;
      handle->count = 0;
    }
    break;
  }

  return return_edge;
}
