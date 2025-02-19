#include "posture_controller_v2.h"

#define BAD_POSTURE_TRANSITION_TIME_MS 3000
#define GOOD_POSTURE_TRANSITION_TIME_MS 3000
#define BAD_POSTURE_HAPTIC_FEEDBACK_INTERVAL_MS 5000
#define UNREALISTIC_POSTURE_THRESHOLD_DEGREES 70.0f

static float good_posture_vector[3];
static float threshold_angle;
static bool is_posture_correct = true;
static uint32_t last_bad_posture_time = 0;
static uint32_t last_good_posture_time = 0;
static uint32_t last_haptic_feedback_time = 0;
void posture_controller_initialize_v2(void)
{
}

void posture_controller_update_v2(void)
{
    uint32_t now = HAL_GetTick();
    accel_data_t accelerometer_vector = accelerometer_controller_get_latest_data();
    normalize_vector(accelerometer_vector);
    float error_angle = get_error_angle(accelerometer_vector);

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
        notify_posture_correct();
        start_calibration_procedure();
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

    if (!is_posture_correct)
    {
        handle_bad_posture();
    }
}

void handle_bad_posture(void)
{
    uint32_t now = HAL_GetTick();
    if (now - last_haptic_feedback_time > BAD_POSTURE_HAPTIC_FEEDBACK_INTERVAL_MS)
    {
        last_haptic_feedback_time = now;
        led_on(100); // Remove this
        haptic_feedback_disable();
        haptic_feedback_play_waveform(1); // Tune this
        haptic_feedback_enable();
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
    
}

void reset_good_posture_vector(void)
{

}