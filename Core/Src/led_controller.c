#include "led_controller.h"
#include "main.h"  // For external references to htim16, etc.

// Extern reference to the TIM16 handle from main.c
extern TIM_HandleTypeDef htim16;

/**
 * @brief Initializes the LED by starting PWM on TIM16_CH1 (PB8).
 */
void led_controller_initialize(void)
{
    // Start the PWM channel
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
}

/**
 * @brief Turns on the LED, scaling the specified percentage (0–100) to a 16-bit PWM duty cycle.
 * @param percent Brightness as a percentage [0..100]
 */
void led_on(uint8_t percent)
{
    if (percent > 100) {
        percent = 100;
    }
    // Map 0..100% to 0..65535
    uint16_t duty_cycle = (uint16_t)((65535UL * percent) / 100UL);

    // Update the CCR (capture compare register) for TIM16 Channel 1
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, duty_cycle);
}

/**
 * @brief Turns off the LED by setting the duty cycle to 0.
 */
void led_off(void)
{
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);
}

/**
 * @brief Executes a blocking LED sequence or pattern.
 * @param sequence The desired sequence to execute.
 */
void led_execute_sequence(led_sequence_t sequence)
{
    switch(sequence)
    {
        case LED_SEQ_NONE:
            // Do nothing
            break;

        case LED_SEQ_THREE_BLINKS:
        {
            // Three quick blinks at ~50% brightness
            for(int i = 0; i < 3; i++)
            {
                led_on(50);
                HAL_Delay(200); // ms ON
                led_off();
                HAL_Delay(200); // ms OFF
            }
        }
        break;

        case LED_SEQ_DOUBLE_BLINK:
        {
            // Two slightly longer blinks at ~80% brightness
            for(int i = 0; i < 2; i++)
            {
                led_on(80);
                HAL_Delay(400);
                led_off();
                HAL_Delay(400);
            }
        }
        break;

        case LED_SEQ_FADE_IN_OUT:
        {
            // Simple fade in from 0% to 100%, then back out
            for(uint8_t val = 0; val <= 100; val++)
            {
                led_on(val);        // increment brightness
                HAL_Delay(10);
            }
            for(uint8_t val = 100; val > 0; val--)
            {
                led_on(val);        // decrement brightness
                HAL_Delay(10);
            }
            led_off();
        }
        break;

        // Add more sequences here as needed

        default:
            // Unrecognized sequence
            break;
    }
}