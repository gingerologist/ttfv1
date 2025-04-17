#include "edge_detect.h"

/* Returns:
 *  1 for rising edge (RESET -> SET)
 * -1 for falling edge (SET -> RESET)
 *  0 for no edge detected
 */
int GPIO_EdgeDetect(SW_HandleTypeDef* handle, GPIO_PinState current_state)
{
    int return_edge = 0;

    switch (handle->status)
    {
    case SW_RELEASED:
        if (current_state == GPIO_PIN_RESET)  // Potential falling edge
        {
            handle->status = SW_DEBOUNCING_DOWN;
            handle->count = 1;
        }
        break;

    case SW_DEBOUNCING_DOWN:
        if (current_state == GPIO_PIN_RESET)
        {
            handle->count++;
            if (handle->count > handle->max_count)  // Configurable debounce threshold
            {
                handle->status = SW_PRESSED;
                handle->count = 0;
                return_edge = -1;  // Falling edge reported here after debounce
            }
        }
        else
        {
            handle->status = SW_RELEASED;
            handle->count = 0;
        }
        break;

    case SW_PRESSED:
        if (current_state == GPIO_PIN_SET)  // Rising edge start
        {
            handle->status = SW_DEBOUNCING_UP;
            handle->count = 1;
        }
        break;

    case SW_DEBOUNCING_UP:
        if (current_state == GPIO_PIN_SET)
        {
            handle->count++;
            if (handle->count > handle->max_count)  // Configurable debounce threshold
            {
                handle->status = SW_RELEASED;
                handle->count = 0;
                return_edge = 1;  // Rising edge reported here after debounce
            }
        }
        else
        {
            handle->status = SW_PRESSED;
            handle->count = 0;
        }
        break;
    }

    return return_edge;
}

#if 0
int GPIO_EdgeDetect(SW_HandleTypeDef* handle, GPIO_PinState current_state)
{
    int return_edge = 0;

    switch (handle->status)
    {
    case SW_RELEASED:
        if (current_state == GPIO_PIN_RESET)  // Falling edge
        {
            handle->status = SW_DEBOUNCING_DOWN;
            handle->count = 1;
            return_edge = -1;
        }
        break;

    case SW_DEBOUNCING_DOWN:
        if (current_state == GPIO_PIN_RESET)
        {
            handle->count++;
            if (handle->count > handle->max_count)  // Configurable debounce threshold
            {
                handle->status = SW_PRESSED;
                handle->count = 0;
            }
        }
        else
        {
            handle->status = SW_RELEASED;
            handle->count = 0;
        }
        break;

    case SW_PRESSED:
        if (current_state == GPIO_PIN_SET)  // Rising edge
        {
            handle->status = SW_DEBOUNCING_UP;
            handle->count = 1;
            return_edge = 1;
        }
        break;

    case SW_DEBOUNCING_UP:
        if (current_state == GPIO_PIN_SET)
        {
            handle->count++;
            if (handle->count > handle->max_count)  // Configurable debounce threshold
            {
                handle->status = SW_RELEASED;
                handle->count = 0;
            }
        }
        else
        {
            handle->status = SW_PRESSED;
            handle->count = 0;
        }
        break;
    }

    return return_edge;
}
#endif

#if 0

// Example usage remains the same, but now with int return type
static SW_HandleTypeDef sw5_handle = {
    .status = SW_RELEASED,
    .count = 0,
    .max_count = 3  // Default debounce
};

// Wrapper function for SW5
static int SW5_EdgeDetect(void)
{
    GPIO_PinState current_state = HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin);
    return GPIO_EdgeDetect(&sw5_handle, current_state);
}

// Example usage in a polling loop
void TaskLoop(void)
{
    while (1)
    {
        int sw5_edge = SW5_EdgeDetect();

        if (sw5_edge == 1)  // Rising edge
        {
            // Do something on rising edge
        }
        else if (sw5_edge == -1)  // Falling edge
        {
            // Do something on falling edge
        }

        // Other tasks...
    }
}

#endif
