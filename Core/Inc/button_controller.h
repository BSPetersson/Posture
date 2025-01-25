#ifndef BUTTON_CONTROLLER_H
#define BUTTON_CONTROLLER_H

#include "stm32f0xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief List of recognized button events.
 *        Expand or adjust as needed.
 */
typedef enum {
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_SINGLE_PRESS,
    BUTTON_EVENT_DOUBLE_PRESS,
    BUTTON_EVENT_TRIPLE_PRESS,
    BUTTON_EVENT_LONG_PRESS
} button_event_t;

/**
 * @brief Initialize the button (PB5). 
 *        Configures internal variables for state tracking.
 *        Assumes MX_GPIO_Init has already enabled EXTI on PB5.
 */
void button_controller_initialize(void);

/**
 * @brief Checks if the button is currently pressed (active low).
 * @return True if pressed, false otherwise.
 */
bool button_is_pressed(void);

/**
 * @brief Periodic function to be called, e.g., in the main loop.
 *        Handles time-based logic (like 3s long press).
 *        Call it at least every 10-20 ms for stable results.
 */
void button_process(void);

/**
 * @brief Returns the most recent button event (single/double/triple/long).
 *        Each call clears the stored event so it won't be reported again.
 *
 * @return The event that was detected since last call, or BUTTON_EVENT_NONE if none.
 */
button_event_t button_get_event(void);

void button_handle_exti(uint16_t GPIO_Pin);

#endif // BUTTON_CONTROLLER_H