#include "posture_controller.h"
#include "haptic_feedback_controller.h"
#include "button_controller.h"
#include "led_controller.h"
#include "main.h"
#include <math.h>

// New constants for the adaptive learning mechanism and variance estimation
#define HISTORY_SIZE                   10
#define MIN_LEARNING_ALPHA             0.01f
#define MAX_LEARNING_ALPHA             0.1f
#define VARIANCE_CONFIDENCE_SCALING    10.0f  // Adjust this to tune how variance maps to learning rate
#define THRESHOLD_VARIANCE_MULTIPLIER    1.0f  // Multiplier when adapting the threshold angle

// Internal state variables
static float measured_vector[3];
static float reference_vector[3];
static float threshold_angle = DEFAULT_THRESHOLD_ANGLE_RAD;
static float current_angle = 0.0f;
static bool is_posture_correct = true;
uint32_t last_bad_posture_time = 0;
uint32_t last_good_posture_time = 0;

// Variables to hold a history of "good" posture angles
static float measured_angle_history[HISTORY_SIZE] = {0};
static int history_index = 0;
static bool history_full = false;

// Variables to manage a correction event.
// When bad posture is detected, we store a snapshot and flag correction_mode.
// Later, when good posture is sustained, we update aggressively.
static bool correction_mode = false;
static float correction_snapshot_vector[3] = {0};

// ---------------------------------------------------------------------------
// normalize_vector: Normalizes a 3D vector in place.
static void normalize_vector(float v[3]) {
    float mag = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (mag > 0.0f) {
        v[0] /= mag;
        v[1] /= mag;
        v[2] /= mag;
    }
}

// ---------------------------------------------------------------------------
// dot_product: Computes the dot product between two vectors.
static float dot_product(const float a[3], const float b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

// ---------------------------------------------------------------------------
// update_angle_history: Adds the current angle to a circular buffer.
static void update_angle_history(float angle) {
    measured_angle_history[history_index] = angle;
    history_index = (history_index + 1) % HISTORY_SIZE;
    if (history_index == 0) {
        history_full = true;
    }
}

// ---------------------------------------------------------------------------
// compute_angle_variance: Calculates the sample variance from the angle history.
// This variance is used as an inverse confidence metric.
static float compute_angle_variance(void) {
    int count = history_full ? HISTORY_SIZE : history_index;
    if (count < 2)
        return 0.0f;
        
    float sum = 0.0f;
    for (int i = 0; i < count; i++) {
        sum += measured_angle_history[i];
    }
    float mean = sum / count;
    
    float sumsq = 0.0f;
    for (int i = 0; i < count; i++) {
        float diff = measured_angle_history[i] - mean;
        sumsq += diff * diff;
    }
    return sumsq / (count - 1);
}

// ---------------------------------------------------------------------------
// compute_adaptive_alpha: Determines the learning factor based on the variance.
// High variance implies low confidence so we update more aggressively.
static float compute_adaptive_alpha(void) {
    float var = compute_angle_variance();
    float adaptive_alpha = MAX_LEARNING_ALPHA;
    if (var > 0.0f) {
        adaptive_alpha = (VARIANCE_CONFIDENCE_SCALING / (var + VARIANCE_CONFIDENCE_SCALING)) * MAX_LEARNING_ALPHA;
    }
    if (adaptive_alpha < MIN_LEARNING_ALPHA)
        adaptive_alpha = MIN_LEARNING_ALPHA;
    if (adaptive_alpha > MAX_LEARNING_ALPHA)
        adaptive_alpha = MAX_LEARNING_ALPHA;
    return adaptive_alpha;
}

// ---------------------------------------------------------------------------
// update_reference_vector: Uses an adaptive exponential moving average to update
// the reference vector. If a correction event is flagged (user just corrected bad posture),
// use the max learning rate to quickly adapt the model.
static void update_reference_vector(void) {
    float effective_alpha = compute_adaptive_alpha();
    if (correction_mode) {
        // Force aggressive update if we detected a correction event.
        effective_alpha = MAX_LEARNING_ALPHA;
        correction_mode = false;
    }
    for (int i = 0; i < 3; i++) {
        reference_vector[i] = (1.0f - effective_alpha) * reference_vector[i] + effective_alpha * measured_vector[i];
    }
    normalize_vector(reference_vector);
}

// ---------------------------------------------------------------------------
// get_angle: Reads accelerometer data, computes the angle between the
// measured vector and reference vector, and logs "good" posture angles.
static float get_angle(void) {
    accel_data_t data = accelerometer_controller_get_latest_data();
    
    measured_vector[0] = data.x_mps2;
    measured_vector[1] = data.y_mps2;
    measured_vector[2] = data.z_mps2;
    normalize_vector(measured_vector);
    
    float dot = dot_product(reference_vector, measured_vector);
    if (dot > 1.0f)
        dot = 1.0f;
    if (dot < -1.0f)
        dot = -1.0f;
    float angle = acosf(dot);
    
    // If the posture is within acceptable limits, record it.
    if (angle <= threshold_angle) {
        update_angle_history(angle);
    }
    
    return angle;
}

// ---------------------------------------------------------------------------
// posture_controller_initialize: Sets the initial reference and resets state.
void posture_controller_initialize(void) {
    threshold_angle = DEFAULT_THRESHOLD_ANGLE_RAD;
    
    accel_data_t data = accelerometer_controller_get_latest_data();
    reference_vector[0] = data.x_mps2;
    reference_vector[1] = data.y_mps2;
    reference_vector[2] = data.z_mps2;
    normalize_vector(reference_vector);
    
    // Reset the history buffer.
    for (int i = 0; i < HISTORY_SIZE; i++) {
        measured_angle_history[i] = 0.0f;
    }
    history_index = 0;
    history_full = false;
    correction_mode = false;
}

// ---------------------------------------------------------------------------
// posture_controller_update: Main non-blocking update function.
// It monitors the angle, detects transitions between good and bad posture,
// initiates correction events, and updates both the reference vector and threshold angle adaptively.
void posture_controller_update(void) {
    uint32_t now = HAL_GetTick();
    current_angle = get_angle();

    // When the angle exceeds the threshold, mark as potential bad posture.
    if (current_angle > threshold_angle) {
        last_bad_posture_time = now;
        // If not already in a correction event, store a snapshot for potential comparison.
        if (!correction_mode) {
            for (int i = 0; i < 3; i++) {
                correction_snapshot_vector[i] = measured_vector[i];
            }
            correction_mode = true;
        }
    } else {
        last_good_posture_time = now;
    }

    // If good posture is sustained after a bad posture event in a time exceeding the defined delay,
    // interpret this as a deliberate correction event. In that case, update the reference vector aggressively.
    if ((last_good_posture_time > last_bad_posture_time) &&
        ((last_good_posture_time - last_bad_posture_time) > POSTURE_GOOD_TRANSITION_TIME_MS) &&
        !is_posture_correct) {
            
        update_reference_vector();
        is_posture_correct = true;
    }

    // If bad posture is sustained beyond its debounce period, update the status.
    if ((last_bad_posture_time > last_good_posture_time) &&
        ((last_bad_posture_time - last_good_posture_time) > POSTURE_BAD_TRANSITION_TIME_MS) &&
        is_posture_correct) {
        is_posture_correct = false;
    }
    
    // Optionally, update threshold_angle adaptively.
    // Here we adjust it toward a target based on the variability (stddev) in the good posture.
    if (is_posture_correct) {
        float adaptive_alpha = compute_adaptive_alpha();
        float stddev = sqrtf(compute_angle_variance());
        float target_threshold = DEFAULT_THRESHOLD_ANGLE_RAD + THRESHOLD_VARIANCE_MULTIPLIER * stddev;
        threshold_angle = (1.0f - adaptive_alpha) * threshold_angle + adaptive_alpha * target_threshold;
    }
}

// ---------------------------------------------------------------------------
// posture_controller_is_posture_correct: Returns true if the current posture
// is within the acceptable threshold. Also disables haptic feedback and LED.
bool posture_controller_is_posture_correct(void) {
    haptic_feedback_disable();
    led_off();
    return is_posture_correct;
}

// ---------------------------------------------------------------------------
// posture_controller_handle_bad_posture: Called to handle sustained bad posture.
// It triggers haptic feedback and LED notifications.
bool posture_controller_handle_bad_posture(void) {
    haptic_feedback_play_waveform(0x01);
    haptic_feedback_start();
    led_on(100);
    return is_posture_correct;
}

// ---------------------------------------------------------------------------
// posture_controller_calibrate: Resets the reference vector to the current accelerometer
// measurement. Optionally, you could also clear the history buffer here.
void posture_controller_calibrate(void) {
    accel_data_t data = accelerometer_controller_get_latest_data();
    reference_vector[0] = data.x_mps2;
    reference_vector[1] = data.y_mps2;
    reference_vector[2] = data.z_mps2;
    normalize_vector(reference_vector);
}