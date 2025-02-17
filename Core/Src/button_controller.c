#include "button_controller.h"

// ------------------------------
// Configuration Constants
// ------------------------------

// Debounce time (ms) for press/release transitions
#define DEBOUNCE_TIME_MS       50U

// Max gap between presses for them to be considered a multi-press
#define MULTI_PRESS_GAP_MS    500U

// Minimum time (ms) for a press to be considered "long"
#define LONG_PRESS_TIME_MS   3000U

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
 */
void button_controller_update(void)
{   
    static uint32_t last_check_time = 0;
    uint32_t now = HAL_GetTick();
    // If we haven't waited enough to avoid excessive checking, skip
    if ((now - last_check_time) < 10U) {
        return;
    }
    last_check_time = now;
    
    bool button_pressed_before = button_pressed;
    button_pressed = read_raw_button_state();

    if (button_pressed_before != button_pressed)
    {
        uint32_t now = HAL_GetTick();
        if ((now - press_start_time) < DEBOUNCE_TIME_MS)
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
    //    check if the pressed duration > LONG_PRESS_TIME_MS
    if (button_pressed && !long_press_reported)
    {
        uint32_t pressed_duration = now - press_start_time;
        if (pressed_duration >= LONG_PRESS_TIME_MS)
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

        if (delta >= MULTI_PRESS_GAP_MS)
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
 */
// void button_handle_exti()
// {
    
// }