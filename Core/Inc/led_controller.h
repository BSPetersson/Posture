#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include "stm32f0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    LED_SEQ_NONE = 0,
    LED_SEQ_THREE_BLINKS,  // Three blinks: on for 200ms, off for 200ms at 50% brightness.
    LED_SEQ_DOUBLE_BLINK,  // Two blinks: on for 400ms, off for 400ms at 80% brightness.
    LED_SEQ_FADE_IN_OUT    // Fade in from 0% to 100% then fade out back to 0%.
    // Add more custom sequences here.
} led_sequence_t;

void led_controller_initialize(void);

void led_controller_update(void);

void led_on(uint8_t percent);

void led_off(void);

void led_execute_sequence(led_sequence_t sequence);

#endif // LED_CONTROLLER_H