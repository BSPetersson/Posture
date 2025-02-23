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
void handle_bad_posture(void);
void notify_posture_correct(void);
void start_calibration_procedure(void);
void handle_initial_still_bad_posture(void);

// Constants
#define BAD_POSTURE_TRANSITION_TIME_MS 3000
#define GOOD_POSTURE_TRANSITION_TIME_MS 1000
#define BAD_POSTURE_HAPTIC_FEEDBACK_INTERVAL_MS 3000
#define UNREALISTIC_POSTURE_THRESHOLD_RADIANS 1.22173f
#define GOOD_POSTURE_VECTOR_RECALIBRATION_TIME_MS 5000
#define GOOD_POSTURE_HISTORY_SIZE 10
#define GOOD_POSTURE_VECTOR_RECALIBRATION_THRESHOLD_RADIANS 0.06
#define BAD_POSTURE_RESET_TIME_MS 180000
#define INITIAL_BAD_POSTURE_THRESHOLD_RADIANS 0.1f
#define INITIAL_BAD_POSTURE_TIME_MS 10000

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

void posture_controller_update(void)
{
    uint32_t now = HAL_GetTick();

    accelerometer_mode_t accelerometer_mode = accelerometer_controller_get_mode();

    if (accelerometer_mode == SUPER_STILL) {
        reset_good_posture_history();
        sleep_controller_activate_sleep_mode();
    }

    if (accelerometer_mode == ACTIVITY) {
        return;
    }

    // Get the latest accelerometer vector
    accel_data_t accelerometer_vector = accelerometer_controller_get_latest_data();
    current_accelerometer_vector[0] = accelerometer_vector.x_mps2;
    current_accelerometer_vector[1] = accelerometer_vector.y_mps2;
    current_accelerometer_vector[2] = accelerometer_vector.z_mps2;

    if (!good_posture_history_full && good_posture_history_index == 0) {
        // Check if the user has remained still for 10 seconds
        if (accelerometer_controller_get_last_measurements_history_max_angle_from_mean() < INITIAL_BAD_POSTURE_THRESHOLD_RADIANS) {
            if ((now - last_bad_posture_time) > INITIAL_BAD_POSTURE_TIME_MS && !did_handle_initial_still_bad_posture) {
                handle_initial_still_bad_posture();
                did_handle_initial_still_bad_posture = true;
            }

            if ((now - last_bad_posture_time) > GOOD_POSTURE_TRANSITION_TIME_MS && did_handle_initial_still_bad_posture) {
                update_good_posture_history(current_accelerometer_vector);
                update_good_posture_vector();
                notify_posture_correct();
            }
        } else {
            last_bad_posture_time = now;
        }
    }
    
    // Check for button long press
    button_event_t button_event = button_controller_get_event();
    if (button_event == BUTTON_EVENT_LONG_PRESS) {
        reset_good_posture_vector();
        notify_posture_correct(); // Provide feedback that recalibration occurred
        did_calibrate = true;
        return;
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
        notify_posture_correct();
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
        
        // Check if bad posture has been maintained for 3 minutes (180000 ms)
        if ((now - last_bad_posture_time) > BAD_POSTURE_RESET_TIME_MS) {
            reset_good_posture_history(); // Reset history if bad posture for 3 minutes
        }
        return;
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
        handle_bad_posture();
        return;
    }
}

void handle_bad_posture(void)
{
    uint32_t now = HAL_GetTick();
    if (now - last_haptic_feedback_time > BAD_POSTURE_HAPTIC_FEEDBACK_INTERVAL_MS)
    {
        last_haptic_feedback_time = now;
        led_on(30);
        haptic_feedback_play_waveform(1);
    }
}

void handle_initial_still_bad_posture(void)
{
    uint32_t now = HAL_GetTick();
    if (now - last_haptic_feedback_time > BAD_POSTURE_HAPTIC_FEEDBACK_INTERVAL_MS)
    {
        last_haptic_feedback_time = now;
        led_on(30);
        haptic_feedback_play_waveform(3);
    }
}

void notify_posture_correct(void)
{
    led_execute_sequence(LED_SEQ_THREE_BLINKS);
    haptic_feedback_play_waveform(2); // Tune this
    led_off();
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
    float sum[3] = {0.0f, 0.0f, 0.0f};
    int count = good_posture_history_full ? GOOD_POSTURE_HISTORY_SIZE : good_posture_history_index;
    for (int i = 0; i < count; i++) {
        for (int j = 0; j < 3; j++) {
            sum[j] += good_posture_history[i][j];
        }
    }
    for (int j = 0; j < 3; j++) {
        good_posture_vector[j] = sum[j] / count;
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
    good_posture_history_index = 0;
    good_posture_history_full = false;
    for (int i = 0; i < GOOD_POSTURE_HISTORY_SIZE; i++) {
        for (int j = 0; j < 3; j++) {
            good_posture_history[i][j] = 0.0f;
        }
    }
}

void posture_controller_init(void) {
    reset_good_posture_history();
}