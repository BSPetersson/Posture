#include "button_controller.h"
#include "parameters.h"
#include "haptic_feedback_controller.h"

// ------------------------------
// Configuration Constants
// ------------------------------
#define BUTTON_RELEASE_MINIMUM_DELAY_MS 30  // Minimum delay before playing release waveform

// ------------------------------
// Internal State Variables
// ------------------------------

static volatile bool button_pressed = false;        // Current stable state

static uint32_t press_start_time = 0;              // Timestamp when button was pressed
static uint32_t last_release_time = 0;             // For single/double/triple press detection

static uint8_t  press_count = 0;                   // How many short presses so far
static bool     long_press_reported = false;       // Did we already report a long press?

// Store the latest button event
static volatile button_event_t latest_event = BUTTON_EVENT_NONE;

// Variables for handling button press/release waveforms
static bool waveform_played_on_press = false;     // Track if press waveform was played
static bool play_release_waveform_pending = false; // Track if release waveform is pending
static uint32_t release_waveform_time = 0;        // Time when release waveform should be played
static uint32_t button_release_time = 0;          // Time when button was last released

// ------------------------------
// Helper Functions
// ------------------------------

/**
 * @brief Read the raw hardware pin state (true = pressed).
 *        PB5 is active low: so pressed = (pin == 0).
 */
static bool read_raw_button_state(void)
{
    // Because the button is active low, invert the read:
    return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET);
}

// ------------------------------
// Public API
// ------------------------------

void button_controller_initialize(void)
{
    // Initialize internal state:
    button_pressed   = read_raw_button_state();
    press_start_time = 0;
    last_release_time = 0;
    press_count       = 0;
    long_press_reported = false;
    latest_event      = BUTTON_EVENT_NONE;
    
    // Initialize waveform state variables
    waveform_played_on_press = false;
    play_release_waveform_pending = false;
    release_waveform_time = 0;
    button_release_time = 0;
}

/**
 * @brief Indicates if the button is currently held down (after debounce).
 */
bool button_is_pressed(void)
{
    return button_pressed;
}

/**
 * @brief Should be called periodically (10–20 ms).
 *        Handles long-press detection and multi-press logic.
 *        Also handles delayed release waveform playback.
 */
void button_controller_update(void)
{   
    uint32_t now = HAL_GetTick();
    
    bool button_pressed_before = button_pressed;
    button_pressed = read_raw_button_state();

    if (button_pressed_before != button_pressed)
    {
        uint32_t now = HAL_GetTick();
        if ((now - press_start_time) < BUTTON_DEBOUNCE_TIME_MS)
        {
            // If we haven't waited enough, skip
            return;
        }
    }

    // If changed from not pressed to pressed, record the time
    if (!button_pressed_before && button_pressed)
    {
        press_start_time = HAL_GetTick();
        press_count++;
    }

    // If changed from pressed to not pressed, record the time
    if (button_pressed_before && !button_pressed)
    {
        last_release_time = HAL_GetTick();
        long_press_reported = false;
    }

    // 1) Long press detection
    //    If the button is pressed and we haven't yet reported a long press,
    //    check if the pressed duration > BUTTON_LONG_PRESS_TIME_MS
    if (button_pressed && !long_press_reported)
    {
        uint32_t pressed_duration = now - press_start_time;
        if (pressed_duration >= BUTTON_LONG_PRESS_TIME_MS)
        {
            long_press_reported = true;
            latest_event = BUTTON_EVENT_LONG_PRESS;
            // Because it's a long press, we reset multi-press logic:
            press_count = 0;
        }
    }

    // 2) Multi-press detection
    //    If the button is currently not pressed, and we have a press_count,
    //    check if enough time has passed since the last release to finalize the event.
    if (!button_pressed && press_count > 0)
    {
        // Time since last release
        uint32_t delta = now - last_release_time;

        if (delta >= BUTTON_MULTI_PRESS_GAP_MS)
        {
            // If we get here, no additional press happened within the multi-press gap
            switch (press_count)
            {
                case 1:
                    latest_event = BUTTON_EVENT_SINGLE_PRESS;
                    break;
                case 2:
                    latest_event = BUTTON_EVENT_DOUBLE_PRESS;
                    break;
                case 3:
                    latest_event = BUTTON_EVENT_TRIPLE_PRESS;
                    break;
                default:
                    // If more presses than 3, do whatever you want:
                    latest_event = BUTTON_EVENT_TRIPLE_PRESS;
                    break;
            }
            press_count = 0; // Reset
        }
    }
    
    // 3) Handle delayed release waveform playback
    if (play_release_waveform_pending && now >= release_waveform_time)
    {
        // Only play the release waveform if the button is not pressed again
        if (!button_pressed)
        {
            haptic_feedback_play_waveform(6);
        }
        play_release_waveform_pending = false;
    }
}

/**
 * @brief Returns the latest detected event and clears it from internal storage.
 */
button_event_t button_controller_get_event(void)
{
    button_event_t ev = latest_event;
    latest_event = BUTTON_EVENT_NONE;
    return ev;
}

// ------------------------------
// EXTI Callback
// ------------------------------
/**
 * @brief Called by HAL when an EXTI interrupt occurs on GPIO pin PB5.
 *        We use this for immediate detection of press/release edges.
 *        Handles playing different waveforms for press and release with timing logic.
 */
void button_handle_exti(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != GPIO_PIN_5)
    {
        return; // Only handle PB5 button
    }
    
    uint32_t now = HAL_GetTick();
    bool current_button_state = read_raw_button_state();
    
    // Button is pressed (pin is LOW)
    if (current_button_state)
    {
        // Cancel any pending release waveform if button is pressed again
        play_release_waveform_pending = false;
        
        // Only play the press waveform if enough time has passed since the last release
        if (now - button_release_time >= BUTTON_RELEASE_MINIMUM_DELAY_MS)
        {
            haptic_feedback_play_waveform(26);
            waveform_played_on_press = true;
        }
        else
        {
            // If pressed too quickly after release, don't play any waveform
            waveform_played_on_press = false;
        }
        
        press_start_time = now;
    }
    // Button is released (pin is HIGH)
    else
    {
        // Record the release time
        button_release_time = now;
        
        if (waveform_played_on_press)
        {
            // Calculate when to play the release waveform
            release_waveform_time = press_start_time + BUTTON_RELEASE_MINIMUM_DELAY_MS;
            
            // If enough time has already passed, play it immediately
            if (now >= release_waveform_time)
            {
                haptic_feedback_play_waveform(6);
            }
            else
            {
                // Otherwise, set the pending flag for later playback
                play_release_waveform_pending = true;
            }
            
            waveform_played_on_press = false;
        }
    }
}