#include "posture_controller.h"
#include "posture_math.h"
#include "accelerometer_controller.h"

// Private function prototypes
// static float dot_product(const float a[3], const float b[3]);
// static void normalize_vector(float v[3]);

// Public function declarations (these match the header file)
float get_angle_between_vectors(float vector1[3], float vector2[3]);
void update_good_posture_vector(void);
void update_good_posture_history(float vector[3]);
void react_to_bad_posture(void);
void react_to_posture_correct(void);
void start_calibration_procedure(void);
void react_to_initial_still_bad_posture(void);

// Constants
#define BAD_POSTURE_TRANSITION_TIME_MS 3000
#define GOOD_POSTURE_TRANSITION_TIME_MS 1000
#define BAD_POSTURE_HAPTIC_FEEDBACK_INTERVAL_MS 3000
#define UNREALISTIC_POSTURE_THRESHOLD_RADIANS 1.22173f
#define GOOD_POSTURE_VECTOR_RECALIBRATION_TIME_MS 5000
#define GOOD_POSTURE_HISTORY_SIZE 10
#define GOOD_POSTURE_VECTOR_RECALIBRATION_THRESHOLD_RADIANS 0.06
#define BAD_POSTURE_RESET_TIME_MS 60000
#define INITIAL_BAD_POSTURE_THRESHOLD_RADIANS 0.1f
#define INITIAL_BAD_POSTURE_TIME_MS 10000
#define POSTURE_RESET_DELAY_MS 1000  // New constant for the delay before resetting posture

float current_accelerometer_vector[3];
float good_posture_vector[3];
float threshold_angle = 0.174533f;
bool is_posture_correct = true;
uint32_t last_bad_posture_time = 0;
uint32_t last_good_posture_time = 0;
uint32_t last_haptic_feedback_time = 0;
bool did_calibrate = false;
float update_alpha = 0.2f;
float good_posture_history[GOOD_POSTURE_HISTORY_SIZE][3];
int good_posture_history_index = 0;
bool good_posture_history_full = false;
bool did_handle_initial_still_bad_posture = false;
static uint32_t still_posture_start_time = 0;
bool device_is_off = false;
bool device_is_shutting_down = false;  // New state to track shutdown process
bool is_resetting_posture = false;     // New flag to track if we're in the process of resetting posture
uint32_t posture_reset_start_time = 0; // New variable to track when the reset started

void posture_controller_update(void)
{
    uint32_t now = HAL_GetTick();

    accelerometer_mode_t accelerometer_mode = accelerometer_controller_get_mode();

    button_event_t button_event = button_controller_get_event();

    if (button_event == BUTTON_EVENT_SINGLE_PRESS) {
        // Instead of resetting immediately, set the flag and start the timer
        is_resetting_posture = true;
        posture_reset_start_time = now;
        did_calibrate = true;
        react_to_calibration();
        return;
    }

    if (button_event == BUTTON_EVENT_LONG_PRESS) {
        if (!device_is_off && !device_is_shutting_down) {
            device_is_shutting_down = true;  // Start shutdown process
            react_to_device_off();           // Run LED sequence
        }
    } else if (button_event != BUTTON_EVENT_NONE && device_is_off) {
        device_is_off = false;
        react_to_device_on();
    }

    // Check if bad posture has been maintained for BAD_POSTURE_RESET_TIME_MS
    if (!is_posture_correct && (now - last_good_posture_time) > BAD_POSTURE_RESET_TIME_MS &&
        !device_is_off && !device_is_shutting_down)
    {
        device_is_shutting_down = true;
        react_to_device_off();
        reset_good_posture_history(); // Reset history if bad posture for BAD_POSTURE_RESET_TIME_MS
    }

    // Check if shutdown sequence is complete
    if (device_is_shutting_down && !led_is_sequence_running()) {
        device_is_shutting_down = false;  // Shutdown sequence complete
        device_is_off = true;             // Now officially off
    }

    // If the device is shutting down, do nothing
    if (device_is_shutting_down) {
        return;
    }

    if (accelerometer_mode == SUPER_STILL || device_is_off) {
        reset_good_posture_history();
        sleep_controller_activate_sleep_mode();
        
        // Check if we were woken up by a button press
        if (sleep_controller_was_woken_by_button()) {
            // We were woken by button, so react as if we received a button event
            device_is_off = false;
            react_to_device_on();
        }
        
        return;
    }

    if (accelerometer_mode == ACTIVITY) {
        return;
    }

    // Get the latest accelerometer vector
    accel_data_t accelerometer_vector = accelerometer_controller_get_latest_data();
    current_accelerometer_vector[0] = accelerometer_vector.x_mps2;
    current_accelerometer_vector[1] = accelerometer_vector.y_mps2;
    current_accelerometer_vector[2] = accelerometer_vector.z_mps2;

    // Check if we're in the process of resetting posture
    if (is_resetting_posture) {
        // If the delay has passed, perform the reset
        if ((now - posture_reset_start_time) >= POSTURE_RESET_DELAY_MS) {
            reset_good_posture_vector();
            is_posture_correct = false;  // Set posture to incorrect after reset
            is_resetting_posture = false;  // Reset the flag
            react_to_posture_correct();  // Play the correct posture waveform
        }
        return;
    }

    // If we haven't filled our good posture history and we're at the start of a new cycle,
    // check if the user has maintained a still posture long enough to use as reference
    if (!good_posture_history_full && good_posture_history_index == 0 && !did_calibrate) {
        // Check if the user has remained still for 10 seconds
        if (accelerometer_controller_get_last_measurements_history_max_angle_from_mean() < INITIAL_BAD_POSTURE_THRESHOLD_RADIANS) {
            uint32_t current_time = HAL_GetTick();
            
            // If this is the first time we're detecting still posture, store the start time
            if (still_posture_start_time == 0) {
                still_posture_start_time = current_time;
            }
            // Check if the still posture has been maintained for the required time
            else if ((current_time - still_posture_start_time) >= INITIAL_BAD_POSTURE_TIME_MS) {
                // Condition fulfilled for the required time, update good posture history and vector
                update_good_posture_history(current_accelerometer_vector);
                update_good_posture_vector();
                still_posture_start_time = 0; // Reset the timer after updating
            }
        } else {
            // Reset the timer if the condition is not met
            still_posture_start_time = 0;
        }
    }

    // Get the angle between the current accelerometer vector and the good posture vector
    float error_angle = get_angle_between_vectors(current_accelerometer_vector, good_posture_vector);

    // If the angle is greater than the unrealistic posture threshold, do nothing
    if (error_angle > UNREALISTIC_POSTURE_THRESHOLD_RADIANS)
    {
        return;
    }

    // If the angle is less than the threshold, update the last good posture time
    // Otherwise, update the last bad posture time
    if (error_angle < threshold_angle)
    {
        last_good_posture_time = now;
    }
    else
    {
        last_bad_posture_time = now;
    }

    // If the last good posture time is greater than the last bad posture time,
    // and the difference is greater than the good posture transition time,
    // then we are in good posture.
    if (last_good_posture_time > last_bad_posture_time &&
        (last_good_posture_time - last_bad_posture_time) > GOOD_POSTURE_TRANSITION_TIME_MS &&
        !is_posture_correct)
    {
        is_posture_correct = true;
        did_calibrate = false;
        react_to_posture_correct();
        return;
    }

    // If the last bad posture time is greater than the last good posture time,
    // and the difference is greater than the bad posture transition time,
    // then we are in bad posture.
    if (last_bad_posture_time > last_good_posture_time &&
        (last_bad_posture_time - last_good_posture_time) > BAD_POSTURE_TRANSITION_TIME_MS &&
        is_posture_correct)
    {
        is_posture_correct = false;
    }

    // If the posture is correct and the last good posture time is greater than the good posture vector recalibration time,
    // and the device have not calibrated yet, then start the calibration procedure.
    if (is_posture_correct &&
        !did_calibrate)
    {
        start_calibration_procedure();
        return;
    }

    if (!is_posture_correct)
    {
        react_to_bad_posture();
        return;
    }
}

void react_to_initial_still_bad_posture(void)
{
    uint32_t now = HAL_GetTick();
    if (now - last_haptic_feedback_time > BAD_POSTURE_HAPTIC_FEEDBACK_INTERVAL_MS)
    {
        last_haptic_feedback_time = now;
        haptic_feedback_play_waveform(3);
    }
}

void start_calibration_procedure(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - last_bad_posture_time) < GOOD_POSTURE_VECTOR_RECALIBRATION_TIME_MS)
    {
        return;
    }

    if (!accelerometer_controller_is_last_measurements_history_full())
    {
        return;
    }

    if (accelerometer_controller_get_last_measurements_history_max_angle_from_mean() > GOOD_POSTURE_VECTOR_RECALIBRATION_THRESHOLD_RADIANS)
    {
        return;
    }

    update_good_posture_history(current_accelerometer_vector);
    
    update_good_posture_vector();

    did_calibrate = true;
}

void reset_good_posture_vector(void)
{
    good_posture_vector[0] = current_accelerometer_vector[0];
    good_posture_vector[1] = current_accelerometer_vector[1];
    good_posture_vector[2] = current_accelerometer_vector[2];
}

void update_good_posture_vector(void)
{
    if (!good_posture_history_full) {
        return;
    }

    float sum[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < GOOD_POSTURE_HISTORY_SIZE; i++) {
        for (int j = 0; j < 3; j++) {
            sum[j] += good_posture_history[i][j];
        }
    }
    for (int j = 0; j < 3; j++) {
        good_posture_vector[j] = sum[j] / GOOD_POSTURE_HISTORY_SIZE;
    }
}

void update_good_posture_history(float vector[3])
{
    for (int i = 0; i < 3; i++) {
        good_posture_history[good_posture_history_index][i] = vector[i];
    }
    good_posture_history_index = (good_posture_history_index + 1) % GOOD_POSTURE_HISTORY_SIZE;
    if (good_posture_history_index == 0)
    {
        good_posture_history_full = true;
    }
}

void reset_good_posture_history(void)
{
    still_posture_start_time = 0;
    good_posture_history_index = 0;
    good_posture_history_full = false;
    for (int i = 0; i < GOOD_POSTURE_HISTORY_SIZE; i++) {
        for (int j = 0; j < 3; j++) {
            good_posture_history[i][j] = 0.0f;
        }
    }
}

void posture_controller_initialize(void) {
    reset_good_posture_history();
}

void react_to_bad_posture(void)
{
    uint32_t now = HAL_GetTick();
    if (now - last_haptic_feedback_time > BAD_POSTURE_HAPTIC_FEEDBACK_INTERVAL_MS)
    {
        last_haptic_feedback_time = now;
        haptic_feedback_play_waveform(75);
    }
}

void react_to_posture_correct(void)
{
    haptic_feedback_play_waveform(12);
}

void react_to_device_off(void) {
    led_execute_sequence(LED_SEQ_FADE_OUT);
    haptic_feedback_play_waveform(10);
}

void react_to_device_on(void) {
    led_execute_sequence(LED_SEQ_FADE_IN);
    haptic_feedback_play_waveform(58);
}

void react_to_calibration(void) {
    led_execute_sequence(LED_SEQ_THREE_BLINKS);
}