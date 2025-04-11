#include "led_controller.h"
#include "main.h"  // For external reference to htim16
#include "parameters.h"
#include <math.h>  // For pow function

// External TIM handle from main.c
extern TIM_HandleTypeDef htim16;

// Gamma correction lookup table
// Using negative gamma for inverse effect - making changes more pronounced at high brightness
#define LED_GAMMA_VALUE 2.2     // Standard gamma value
#define GAMMA_TABLE_SIZE 101    // 0-100%
static uint16_t gamma_table[GAMMA_TABLE_SIZE];

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
 * @brief Internal function to initialize the gamma correction table
 */
static void init_gamma_table(void)
{
    for (int i = 0; i < GAMMA_TABLE_SIZE; i++) {
        // Normalize brightness to 0.0-1.0
        float normalized = (float)i / (float)(GAMMA_TABLE_SIZE - 1);

        float corrected = 1.0f - powf(1.0f - normalized, 1.0f / LED_GAMMA_VALUE);
        
        // Scale to uint16_t range (0-65535) for PWM
        gamma_table[i] = (uint16_t)(corrected * 65535.0f);
    }
}

/**
 * @brief Internal function: update PWM duty cycle from percentage.
 */
static void update_pwm(uint8_t percent)
{
    if (percent > 100) {
        percent = 100;
    }
    
    // Use the gamma-corrected value from lookup table
    uint16_t duty_cycle = gamma_table[percent];
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, duty_cycle);
}

void led_controller_initialize(void)
{
    // Initialize gamma table
    init_gamma_table();
    
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
    } else if (sequence == LED_SEQ_FADE_IN) {
        led_state.brightness = 0;
        led_state.fade_direction = 1;
    } else if (sequence == LED_SEQ_FADE_OUT) {
        led_state.brightness = LED_FADE_MAX_BRIGHTNESS;
        led_state.fade_direction = -1;
    }
    // Set next update time to start immediately.
    led_state.next_update = HAL_GetTick();
}

bool led_is_sequence_running(void)
{
    return led_state.active;
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
            if (now >= led_state.next_update)
            {
                if (led_state.substate == 0)
                {
                    update_pwm(LED_THREE_BLINKS_BRIGHTNESS);
                    led_state.next_update = now + LED_THREE_BLINKS_ON_TIME_MS;
                    led_state.substate = 1;
                }
                else // substate == 1
                {
                    update_pwm(0);
                    led_state.next_update = now + LED_THREE_BLINKS_OFF_TIME_MS;
                    led_state.substate = 0;
                    led_state.current_blink++;
                    if (led_state.current_blink >= LED_THREE_BLINKS_COUNT)
                    {
                        led_state.sequence = LED_SEQ_NONE;
                        led_state.active = false;
                    }
                }
            }
            break;

        case LED_SEQ_DOUBLE_BLINK:
            if (now >= led_state.next_update)
            {
                if (led_state.substate == 0)
                {
                    update_pwm(LED_DOUBLE_BLINK_BRIGHTNESS);
                    led_state.next_update = now + LED_DOUBLE_BLINK_ON_TIME_MS;
                    led_state.substate = 1;
                }
                else // substate == 1
                {
                    update_pwm(0);
                    led_state.next_update = now + LED_DOUBLE_BLINK_OFF_TIME_MS;
                    led_state.substate = 0;
                    led_state.current_blink++;
                    if (led_state.current_blink >= LED_DOUBLE_BLINK_COUNT)
                    {
                        led_state.sequence = LED_SEQ_NONE;
                        led_state.active = false;
                    }
                }
            }
            break;

        case LED_SEQ_FADE_IN_OUT:
            if (now >= led_state.next_update)
            {
                update_pwm(led_state.brightness);
                led_state.next_update = now + LED_FADE_UPDATE_TIME_MS;
                led_state.brightness += led_state.fade_direction;
                if (led_state.brightness >= LED_FADE_MAX_BRIGHTNESS && led_state.fade_direction > 0)
                {
                    led_state.brightness = LED_FADE_MAX_BRIGHTNESS;
                    led_state.fade_direction = -1;
                }
                else if (led_state.brightness == 0 && led_state.fade_direction < 0)
                {
                    update_pwm(0);
                    led_state.sequence = LED_SEQ_NONE;
                    led_state.active = false;
                }
            }
            break;
            
        case LED_SEQ_FADE_IN:
            if (now >= led_state.next_update)
            {
                update_pwm(led_state.brightness);
                
                // Calculate step size based on the total duration
                // For a smooth fade over LED_FADE_DURATION_MS, we determine how many steps to take
                uint8_t total_steps = LED_FADE_MAX_BRIGHTNESS; // 0 to 100
                uint16_t time_per_step = LED_FADE_DURATION_MS / total_steps;
                
                // Ensure time_per_step is at least LED_FADE_UPDATE_TIME_MS
                if (time_per_step < LED_FADE_UPDATE_TIME_MS) {
                    time_per_step = LED_FADE_UPDATE_TIME_MS;
                }
                
                led_state.next_update = now + time_per_step;
                led_state.brightness += 1; // Increase by 1 each time
                
                if (led_state.brightness >= LED_FADE_MAX_BRIGHTNESS)
                {
                    // We've reached maximum brightness
                    // Hold at max brightness briefly before turning off
                    led_state.brightness = LED_FADE_MAX_BRIGHTNESS;
                    update_pwm(led_state.brightness);
                    
                    // Turn off the LED
                    led_off();
                    led_state.sequence = LED_SEQ_NONE;
                    led_state.active = false;
                }
            }
            break;
            
        case LED_SEQ_FADE_OUT:
            if (now >= led_state.next_update)
            {
                update_pwm(led_state.brightness);
                
                // Calculate step size based on the total duration
                uint8_t total_steps = LED_FADE_MAX_BRIGHTNESS; // 100 to 0
                uint16_t time_per_step = LED_FADE_DURATION_MS / total_steps;
                
                // Ensure time_per_step is at least LED_FADE_UPDATE_TIME_MS
                if (time_per_step < LED_FADE_UPDATE_TIME_MS) {
                    time_per_step = LED_FADE_UPDATE_TIME_MS;
                }
                
                led_state.next_update = now + time_per_step;
                led_state.brightness -= 1; // Decrease by 1 each time
                
                if (led_state.brightness == 0 || led_state.brightness > LED_FADE_MAX_BRIGHTNESS) // Check for underflow
                {
                    // We've reached minimum brightness, end the sequence
                    led_state.brightness = 0;
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