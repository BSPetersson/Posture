#include "led_controller.h"
#include "main.h"  // For external reference to htim16

// External TIM handle from main.c
extern TIM_HandleTypeDef htim16;

// Internal state structure for non-blocking LED sequences.
typedef struct {
    led_sequence_t sequence;   // Current sequence.
    bool active;               // True if a sequence is running.
    uint32_t next_update;      // Next update time (in ms from HAL_GetTick()).
    uint8_t current_blink;     // Number of blinks completed (for blink sequences).
    uint8_t substate;          // Used to differentiate phases (e.g. 0 = off, 1 = on).
    // Variables for fade in/out sequence:
    uint8_t brightness;        // Current brightness (0-100).
    int8_t fade_direction;     // +1 for fading in, -1 for fading out.
} led_state_t;

static led_state_t led_state;

/**
 * @brief Internal function: update PWM duty cycle from percentage.
 */
static void update_pwm(uint8_t percent)
{
    if (percent > 100) {
        percent = 100;
    }
    // Map 0..100% to 0..65535.
    uint16_t duty_cycle = (uint16_t)((65535UL * percent) / 100UL);
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, duty_cycle);
}

void led_controller_initialize(void)
{
    // Start PWM on TIM16 Channel 1.
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

    // Initialize state.
    led_state.sequence      = LED_SEQ_NONE;
    led_state.active        = false;
    led_state.next_update   = 0;
    led_state.current_blink = 0;
    led_state.substate      = 0;
    led_state.brightness    = 0;
    led_state.fade_direction = 1;
}

void led_on(uint8_t percent)
{
    // Immediately set LED brightness.
    update_pwm(percent);
    // Optionally, if you want to cancel any running sequence on manual control:
    // led_state.active = false;
    // led_state.sequence = LED_SEQ_NONE;
}

void led_off(void)
{
    update_pwm(0);
    // Optionally cancel any running sequence if manual off is desired.
    // led_state.active = false;
    // led_state.sequence = LED_SEQ_NONE;
}

void led_execute_sequence(led_sequence_t sequence)
{
    // Set the desired sequence.
    led_state.sequence = sequence;
    led_state.active = true;
    led_state.current_blink = 0;
    led_state.substate = 0;
    // For fade sequence, initialize brightness and direction.
    if (sequence == LED_SEQ_FADE_IN_OUT) {
        led_state.brightness = 0;
        led_state.fade_direction = 1;
    }
    // Set next update time to start immediately.
    led_state.next_update = HAL_GetTick();
}

void led_controller_update(void)
{
    if (!led_state.active || led_state.sequence == LED_SEQ_NONE)
    {
        return;
    }

    uint32_t now = HAL_GetTick();

    switch (led_state.sequence)
    {
        case LED_SEQ_THREE_BLINKS:
            // Blink: 200ms on at 50%, 200ms off.
            if (now >= led_state.next_update)
            {
                if (led_state.substate == 0)
                {
                    // Turn LED on at 50%.
                    update_pwm(50);
                    led_state.next_update = now + 200;
                    led_state.substate = 1;
                }
                else // substate == 1
                {
                    // Turn LED off.
                    update_pwm(0);
                    led_state.next_update = now + 200;
                    led_state.substate = 0;
                    led_state.current_blink++;
                    if (led_state.current_blink >= 3)
                    {
                        // Sequence complete.
                        led_state.sequence = LED_SEQ_NONE;
                        led_state.active = false;
                    }
                }
            }
            break;

        case LED_SEQ_DOUBLE_BLINK:
            // Blink: 400ms on at 80%, 400ms off.
            if (now >= led_state.next_update)
            {
                if (led_state.substate == 0)
                {
                    update_pwm(80);
                    led_state.next_update = now + 400;
                    led_state.substate = 1;
                }
                else // substate == 1
                {
                    update_pwm(0);
                    led_state.next_update = now + 400;
                    led_state.substate = 0;
                    led_state.current_blink++;
                    if (led_state.current_blink >= 2)
                    {
                        led_state.sequence = LED_SEQ_NONE;
                        led_state.active = false;
                    }
                }
            }
            break;

        case LED_SEQ_FADE_IN_OUT:
            // Fade in from 0 to 100, then fade out back to 0.
            if (now >= led_state.next_update)
            {
                update_pwm(led_state.brightness);
                led_state.next_update = now + 10;  // Update every 10ms.
                // Update brightness.
                led_state.brightness += led_state.fade_direction;
                // Check bounds.
                if (led_state.brightness >= 100 && led_state.fade_direction > 0)
                {
                    led_state.brightness = 100;
                    led_state.fade_direction = -1;
                }
                else if (led_state.brightness == 0 && led_state.fade_direction < 0)
                {
                    // Sequence complete.
                    update_pwm(0);
                    led_state.sequence = LED_SEQ_NONE;
                    led_state.active = false;
                }
            }
            break;

        default:
            // Unknown sequence; do nothing.
            break;
    }
}