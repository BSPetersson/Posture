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
static volatile bool button_changed = false;        // Track if pin just changed

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
    button_changed   = false;
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
void button_process(void)
{
    static uint32_t last_check_time = 0;
    uint32_t now = HAL_GetTick();

    // If we haven't waited enough to avoid excessive checking, skip
    if ((now - last_check_time) < 10U)
        return;
    last_check_time = now;

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
button_event_t button_get_event(void)
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
void button_handle_exti(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != GPIO_PIN_5) {
        return;  // Not our button
    }

    // Read the raw hardware state
    bool raw_state = read_raw_button_state();

    // Debounce logic: we skip if the state is the same as we have recorded
    // but let's store a timestamp and do a minimal check
    static uint32_t last_interrupt_time = 0;
    uint32_t now = HAL_GetTick();

    // If the interrupt occurred too soon after the last one, ignore (debouncing)
    if ((now - last_interrupt_time) < DEBOUNCE_TIME_MS)
    {
        return;
    }
    last_interrupt_time = now;

    // If we detect a change from not pressed -> pressed
    if (!button_pressed && raw_state)
    {
        // Pressed
        button_pressed = true;
        press_start_time = now;
        long_press_reported = false;
        // Increment the press_count (for multi-press logic),
        // but only if we haven't reported a long press yet.
        press_count++;
    }
    else if (button_pressed && !raw_state)
    {
        // Released
        button_pressed = false;
        last_release_time = now;

        // If we released but the press was short (< LONG_PRESS_TIME_MS),
        // we rely on button_process() to figure out single/double/triple press
    }
}