#include "button_controller.h"
#include "parameters.h"
#include "haptic_feedback_controller.h"
#include "stm32f0xx_hal.h"  // Add HAL header for GPIO definitions

// ------------------------------
// Configuration Constants
// ------------------------------
#define BUTTON_RELEASE_MINIMUM_DELAY_MS 60  // Minimum delay before playing release waveform
#define BUTTON_LOCKOUT_PERIOD_MS 70         // Period to ignore new presses after a press/release cycle
#define WAVEFORM_PLAY_TIME_MS 50             // Estimated time for a waveform to finish playing
#define DEBOUNCE_TIME_MS 50                  // Time to wait for button to stabilize
#define SINGLE_PRESS_MAX_DURATION_MS 500     // Maximum duration for a single press (not a long press)

// ------------------------------
// Button State Machine
// ------------------------------
typedef enum {
    BUTTON_STATE_IDLE,           // Button is stable in the released state
    BUTTON_STATE_DEBOUNCE_DOWN,  // Button was just pressed, waiting for bounce to settle
    BUTTON_STATE_PRESSED,        // Button is stable in the pressed state
    BUTTON_STATE_DEBOUNCE_UP,    // Button was just released, waiting for bounce to settle
} button_state_t;

// ------------------------------
// Internal State Variables
// ------------------------------

static button_state_t button_state = BUTTON_STATE_IDLE;  // Current state in the state machine
static uint32_t state_change_time = 0;                  // When the state last changed
static bool button_pressed = false;                      // Current stable state (after debouncing)

static uint32_t press_start_time = 0;                   // Timestamp when button was pressed
static uint32_t last_release_time = 0;                  // For single press detection

static bool     long_press_reported = false;            // Did we already report a long press?
static bool     single_press_detected = false;          // Did we detect a single press?

// Store the latest button event
static volatile button_event_t latest_event = BUTTON_EVENT_NONE;

// Variables for handling button press/release waveforms
static bool waveform_played_on_press = false;           // Track if press waveform was played
static bool play_release_waveform_pending = false;      // Track if release waveform is pending
static uint32_t release_waveform_time = 0;              // Time when release waveform should be played
static uint32_t button_release_time = 0;                // Time when button was last released
static uint32_t lockout_end_time = 0;                   // Time when the lockout period ends
static uint32_t press_waveform_end_time = 0;            // Estimated time when press waveform will finish playing
static bool release_waveform_played = false;             // Track if release waveform has been played

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

/**
 * @brief Check if we're in the lockout period after a button press/release cycle
 * @param now Current time in milliseconds
 * @return true if in lockout period, false otherwise
 */
static bool is_in_lockout_period(uint32_t now)
{
    return (now < lockout_end_time);
}

/**
 * @brief Check if the press waveform has finished playing
 * @param now Current time in milliseconds
 * @return true if press waveform has finished playing, false otherwise
 */
static bool has_press_waveform_finished(uint32_t now)
{
    return (now >= press_waveform_end_time);
}

/**
 * @brief Check if a press qualifies as a single press (not too short, not too long)
 * @param press_duration Duration of the press in milliseconds
 * @return true if it's a valid single press, false otherwise
 */
static bool is_valid_single_press(uint32_t press_duration)
{
    // Must be longer than debounce time but shorter than long press time
    return (press_duration >= DEBOUNCE_TIME_MS && 
            press_duration < BUTTON_LONG_PRESS_TIME_MS &&
            press_duration < SINGLE_PRESS_MAX_DURATION_MS);
}

/**
 * @brief Update the button state machine
 * @param now Current time in milliseconds
 * @param raw_state Current raw button state (true = pressed)
 */
static void update_button_state_machine(uint32_t now, bool raw_state)
{
    // Calculate time since last state change
    uint32_t time_in_state = now - state_change_time;
    
    // State machine transitions
    switch (button_state)
    {
        case BUTTON_STATE_IDLE:
            // If button is pressed, move to debounce down state
            if (raw_state)
            {
                button_state = BUTTON_STATE_DEBOUNCE_DOWN;
                state_change_time = now;
                // Reset button press detection flags when a new press starts
                single_press_detected = false;
                long_press_reported = false;
            }
            break;
            
        case BUTTON_STATE_DEBOUNCE_DOWN:
            // If button is released during debounce, go back to idle
            if (!raw_state)
            {
                button_state = BUTTON_STATE_IDLE;
                state_change_time = now;
            }
            // If debounce time has passed and button is still pressed, move to pressed state
            else if (time_in_state >= DEBOUNCE_TIME_MS)
            {
                button_state = BUTTON_STATE_PRESSED;
                state_change_time = now;
                button_pressed = true;
                press_start_time = now;
            }
            break;
            
        case BUTTON_STATE_PRESSED:
            // If button is released, move to debounce up state
            if (!raw_state)
            {
                button_state = BUTTON_STATE_DEBOUNCE_UP;
                state_change_time = now;
            }
            break;
            
        case BUTTON_STATE_DEBOUNCE_UP:
            // If button is pressed during debounce, go back to pressed state
            if (raw_state)
            {
                button_state = BUTTON_STATE_PRESSED;
                state_change_time = now;
            }
            // If debounce time has passed and button is still released, move to idle state
            else if (time_in_state >= DEBOUNCE_TIME_MS)
            {
                button_state = BUTTON_STATE_IDLE;
                state_change_time = now;
                button_pressed = false;
                last_release_time = now;
                
                // Check if this was a valid single press
                uint32_t press_duration = now - press_start_time;
                if (is_valid_single_press(press_duration) && !long_press_reported)
                {
                    single_press_detected = true;
                    latest_event = BUTTON_EVENT_SINGLE_PRESS;
                }
            }
            break;
    }
}

// ------------------------------
// Public API
// ------------------------------

void button_controller_initialize(void)
{
    // Initialize internal state:
    button_state = BUTTON_STATE_IDLE;
    state_change_time = HAL_GetTick();
    button_pressed = false;
    press_start_time = 0;
    last_release_time = 0;
    long_press_reported = false;
    single_press_detected = false;
    latest_event = BUTTON_EVENT_NONE;
    
    // Initialize waveform state variables
    waveform_played_on_press = false;
    play_release_waveform_pending = false;
    release_waveform_time = 0;
    button_release_time = 0;
    lockout_end_time = 0;
    press_waveform_end_time = 0;
    release_waveform_played = false;
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
 *        Handles long-press detection.
 *        Also handles delayed release waveform playback.
 */
void button_controller_update(void)
{   
    uint32_t now = HAL_GetTick();
    
    // Update the button state machine with the current raw button state
    bool raw_button_state = read_raw_button_state();
    update_button_state_machine(now, raw_button_state);
    
    // Long press detection
    // If the button is pressed and we haven't yet reported a long press,
    // check if the pressed duration > BUTTON_LONG_PRESS_TIME_MS
    if (button_pressed && !long_press_reported)
    {
        uint32_t pressed_duration = now - press_start_time;
        if (pressed_duration >= BUTTON_LONG_PRESS_TIME_MS)
        {
            long_press_reported = true;
            latest_event = BUTTON_EVENT_LONG_PRESS;
        }
    }
    
    // Handle delayed release waveform playback
    if (play_release_waveform_pending && now >= release_waveform_time && !release_waveform_played)
    {
        // Only play the release waveform if:
        // 1. The button is not pressed again
        // 2. The press waveform has finished playing
        if (!button_pressed && has_press_waveform_finished(now))
        {
            haptic_feedback_play_waveform(6);
            release_waveform_played = true;
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
        // Check if we're in the lockout period
        if (is_in_lockout_period(now))
        {
            // Ignore this press event
            return;
        }
        
        // Only process if we're transitioning from not pressed to pressed
        if (button_state == BUTTON_STATE_IDLE || button_state == BUTTON_STATE_DEBOUNCE_UP)
        {
            // Reset button press detection flags when a new press starts
            single_press_detected = false;
            long_press_reported = false;
            
            // Cancel any pending release waveform if button is pressed again
            play_release_waveform_pending = false;
            release_waveform_played = false;
            
            // Play the press waveform
            haptic_feedback_play_waveform(26);
            waveform_played_on_press = true;
            
            // Record the estimated end time of the press waveform
            press_waveform_end_time = now + WAVEFORM_PLAY_TIME_MS;
            
            // Update state machine
            button_state = BUTTON_STATE_DEBOUNCE_DOWN;
            state_change_time = now;
        }
    }
    // Button is released (pin is HIGH)
    else
    {
        // Only process if we're transitioning from pressed to not pressed
        if (button_state == BUTTON_STATE_PRESSED || button_state == BUTTON_STATE_DEBOUNCE_DOWN)
        {
            // Record the release time
            button_release_time = now;
            
            if (waveform_played_on_press && !release_waveform_played)
            {
                // Calculate when to play the release waveform
                // Make sure it's after both the minimum delay AND the press waveform has finished
                uint32_t min_release_time = press_start_time + BUTTON_RELEASE_MINIMUM_DELAY_MS;
                uint32_t waveform_end_time = press_waveform_end_time;
                
                // Use the later of the two times
                release_waveform_time = (min_release_time > waveform_end_time) ? 
                                       min_release_time : waveform_end_time;
                
                // If enough time has already passed, play it immediately
                if (now >= release_waveform_time)
                {
                    haptic_feedback_play_waveform(6);
                    release_waveform_played = true;
                }
                else
                {
                    // Otherwise, set the pending flag for later playback
                    play_release_waveform_pending = true;
                }
                
                waveform_played_on_press = false;
                
                // Set the lockout period to prevent rapid toggling
                lockout_end_time = now + BUTTON_LOCKOUT_PERIOD_MS;
            }
            
            // Update state machine
            button_state = BUTTON_STATE_DEBOUNCE_UP;
            state_change_time = now;
        }
    }
}