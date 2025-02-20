/*
The Algorithm
There should be a good_posture_vector that is representing the ideal posture

If the angle deviates more than x degrees for more than x seconds, then it should go into realign mode

While in realign mode, it should vibrate. There should be x seconds between each vibration

If it is over the threshold by an unrealistic amount, it should not notify

If the posture returns under the threshold for more than x seconds, it should go back into good_posture

There should be a function for asking if the posture is good

Learning the good_posture_vector while correcting
There are two parameters that can be tuned, the good_posture_vector and the threshold

We can update/learn the good_posture_vector dynamically when the posture goes from bad to good.
If:
- the posture returns from bad to good
- it have been under the threshold for x seconds
- The max distance between any two out of the last x measurements are under x deg
This ensures that we know they corrected their posture, and that they are sitting still
Then update the good_posture_vector by moving it closer to the average of the last x measurements

Learning the good_posture_vector after device have been placed
If the device have just been placed on the body or if it have been taken off and placed again, the good_posture_vector could be totally off. We need to figure out a way to know if this is the case
If we have remained outside the threshold for a very long time, we might assume that good_posture_vector is no longer valid.
If the angle is over threshold by an extreme amount for a long time, we can assume it is not valid anymore
If the user does not correct the posture when notified, we can assume that they are already at the correct posture

The good_posture_vector should be reset if:
- It have been over the threshold for over x seconds

If it transitions from moving a lot mode to not moving a lot mode, then we don't want to notify the use of baad posture until they have been inside the threshold. This is because, we can assume that the device is moving a lot while being placed and then we want to give it time to recalibrate without notifying of bad posture.

Actually, if it have been moving a lot, then we want to recalibrate no matter what, so we will not notify until at have recalibrated. It should recalibrate x seconds after movement becomes more stable

Calibrate with button
If the button is pressed for more than 3 seconds, just recalibrate

Learning the threshold
The more confident we are about the correct posture vector, the smaller we can make the threshold.
We should mare sure not to make is too small to get false positives and not to bit so we never detect a bad posture

There should be a min and max threshold

We could keep a buffer of the last x samples from when the posture is corrected dynamically.
Then we could find a variance from the and use that to scale the threshold in a way where the larger the variance, the bigger the threshold
We might even wight the newer samples more when computing the variance
*/
#include "posture_controller_v2.h"

#define BAD_POSTURE_TRANSITION_TIME_MS 3000
#define GOOD_POSTURE_TRANSITION_TIME_MS 3000
#define BAD_POSTURE_HAPTIC_FEEDBACK_INTERVAL_MS 5000
#define UNREALISTIC_POSTURE_THRESHOLD_DEGREES 70.0f
#define GOOD_POSTURE_VECTOR_RECALIBRATION_TIME_MS 10000
#define GOOD_POSTURE_HISTORY_SIZE 10
#define LAST_MEASUREMENTS_HISTORY_SIZE 100
#define GOOD_POSTURE_VECTOR_RECALIBRATION_THRESHOLD_DEGREES 5.0f

static float current_accelerometer_vector[3];
static float good_posture_vector[3];
static float threshold_angle;
static bool is_posture_correct = true;
static uint32_t last_bad_posture_time = 0;
static uint32_t last_good_posture_time = 0;
static uint32_t last_haptic_feedback_time = 0;
static bool did_calibrate = false;
static float update_alpha = 0.2f;
static float good_posture_history[GOOD_POSTURE_HISTORY_SIZE][3];
static float last_measurements_history[LAST_MEASUREMENTS_HISTORY_SIZE][3];
static int last_measurements_history_index = 0;
static bool last_measurements_history_full = false;
static int good_posture_history_index = 0;
static bool good_posture_history_full = false;

void posture_controller_update_v2(void)
{
    uint32_t now = HAL_GetTick();
    
    // Get the latest accelerometer vector
    accel_data_t accelerometer_vector = accelerometer_controller_get_latest_data();
    current_accelerometer_vector[0] = accelerometer_vector.x_mps2;
    current_accelerometer_vector[1] = accelerometer_vector.y_mps2;
    current_accelerometer_vector[2] = accelerometer_vector.z_mps2;
    normalize_vector(current_accelerometer_vector);

    // Get the angle between the current accelerometer vector and the good posture vector
    float error_angle = get_angle_between_vectors(current_accelerometer_vector, good_posture_vector);

    // If the angle is greater than the unrealistic posture threshold, do nothing
    if (error_angle > UNREALISTIC_POSTURE_THRESHOLD_DEGREES)
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
        (now - last_good_posture_time) > GOOD_POSTURE_VECTOR_RECALIBRATION_TIME_MS &&
        !did_calibrate)
    {
        start_calibration_procedure();
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
        haptic_feedback_play_waveform(1); // Tune this
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
    if (!last_measurements_history_full)
    {
        return;
    }

    // Calculate average vector
    float avg_vector[3] = {0, 0, 0};
    int array_size = LAST_MEASUREMENTS_HISTORY_SIZE;
    
    for (int i = 0; i < array_size; i++) {
        avg_vector[0] += last_measurements_history[i][0];
        avg_vector[1] += last_measurements_history[i][1];
        avg_vector[2] += last_measurements_history[i][2];
    }
    
    avg_vector[0] /= array_size;
    avg_vector[1] /= array_size;
    avg_vector[2] /= array_size;
    normalize_vector(avg_vector);

    // Find maximum deviation from average
    float max_deviation = 0.0f;
    for (int i = 0; i < array_size; i++) {
        float angle = get_angle_between_vectors(last_measurements_history[i], avg_vector);
        if (angle > max_deviation) {
            max_deviation = angle;
        }
    }

    if (max_deviation > GOOD_POSTURE_VECTOR_RECALIBRATION_THRESHOLD_DEGREES)
    {
        return;
    }

    update_good_posture_history(avg_vector);
    
    update_good_posture_vector(avg_vector);

    did_calibrate = true;
}

void reset_good_posture_vector(void)
{
    good_posture_vector[0] = current_accelerometer_vector[0];
    good_posture_vector[1] = current_accelerometer_vector[1];
    good_posture_vector[2] = current_accelerometer_vector[2];
    normalize_vector(good_posture_vector);
}

bool posture_controller_is_posture_correct(void)
{
    return is_posture_correct;
}

void update_good_posture_vector(float vector[3])
{
    for (int i = 0; i < 3; i++) {
        good_posture_vector[i] = (1.0f - update_alpha) * good_posture_vector[i] + update_alpha * vector[i];
    }
    normalize_vector(good_posture_vector);
}

float get_angle_between_vectors(float vector1[3], float vector2[3])
{
    float dot = dot_product(vector1, vector2);
    if (dot > 1.0f)
        dot = 1.0f;
    if (dot < -1.0f)
        dot = -1.0f;
    return acosf(dot);
}

static float dot_product(const float a[3], const float b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static void normalize_vector(float v[3]) {
    float mag = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (mag > 0.0f) {
        v[0] /= mag;
        v[1] /= mag;
        v[2] /= mag;
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