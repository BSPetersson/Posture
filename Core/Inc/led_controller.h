#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include "stm32f0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initializes the LED (starts PWM on TIM16_CH1).
 */
void led_controller_initialize(void);

/**
 * @brief Turns on the LED at the specified brightness percentage.
 * @param percent A value from 0 to 100.
 *                - 0% = off
 *                - 100% = maximum brightness
 */
void led_on(uint8_t percent);

/**
 * @brief Turns off the LED.
 */
void led_off(void);

/**
 * @brief Enumeration of predefined LED sequences.
 */
typedef enum {
    LED_SEQ_NONE = 0,
    LED_SEQ_THREE_BLINKS,
    LED_SEQ_DOUBLE_BLINK,
    LED_SEQ_FADE_IN_OUT,
    // Add more custom sequences here
} led_sequence_t;

/**
 * @brief Executes a predefined sequence/pattern in a blocking manner.
 * @param sequence The sequence to execute.
 */
void led_execute_sequence(led_sequence_t sequence);

#endif // LED_CONTROLLER_H