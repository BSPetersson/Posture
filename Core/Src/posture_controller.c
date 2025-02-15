#include "posture_controller.h"
#include "haptic_feedback_controller.h"
#include "button_controller.h"
#include "led_controller.h"
#include "main.h"
#include <math.h>

// Constants
#define DEFAULT_THRESHOLD_ANGLE_RAD (15.0f * (M_PI / 180.0f))  // 15 degrees in radians
#define STABLE_DURATION_MS 2000U  // 2 seconds required for a stable correction
#define EMA_ALPHA 0.2f          // Weight for exponential moving average update for the reference

// Internal state variables
static posture_state_t posture_state;
static float reference_vector[3];  // normalized reference posture vector
static float threshold_angle = DEFAULT_THRESHOLD_ANGLE_RAD;
static uint32_t stable_start_time = 0; // time when the user has been stable
static uint32_t alert_start_time = 0; // time when the alert started

// Helper: Normalize a 3D vector in place.
static void normalize_vector(float v[3]) {
    float mag = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (mag > 0.0f) {
        v[0] /= mag;
        v[1] /= mag;
        v[2] /= mag;
    }
}

// Helper: Compute the dot product of two 3D vectors.
static float dot_product(const float a[3], const float b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

// Helper: Update the reference posture vector using an exponential moving average.
static void update_reference_vector(const float measured[3]) {
    for (int i = 0; i < 3; i++) {
        reference_vector[i] = (1.0f - EMA_ALPHA) * reference_vector[i] + EMA_ALPHA * measured[i];
    }
    normalize_vector(reference_vector);
}

// Initialization: Set the initial reference posture from the current accelerometer reading.
void posture_controller_initialize(void) {
    accel_data_t data;
    if (accelerometer_read_mps2(&data) == HAL_OK) {
        reference_vector[0] = data.x_mps2;
        reference_vector[1] = data.y_mps2;
        reference_vector[2] = data.z_mps2;
        normalize_vector(reference_vector);
    } else {
        // If reading fails, assume upright posture (gravity along Z).
        reference_vector[0] = 0.0f;
        reference_vector[1] = 0.0f;
        reference_vector[2] = 1.0f;
    }
    threshold_angle = DEFAULT_THRESHOLD_ANGLE_RAD;
    posture_state = POSTURE_STATE_MONITORING;
    stable_start_time = 0;
    alert_start_time = 0;
}

// Main update function: Call this periodically (e.g., in your main loop).
void posture_controller_update(void) {
    accel_data_t data;
    if (accelerometer_read_mps2(&data) != HAL_OK) {
        return; // Skip update if the accelerometer reading fails.
    }
    
    // Create a measured vector from accelerometer data and normalize it.
    float measured[3] = { data.x_mps2, data.y_mps2, data.z_mps2 };
    normalize_vector(measured);
    
    // Compute the angular deviation between the measured and reference posture.
    float dot = dot_product(reference_vector, measured);
    if (dot > 1.0f) dot = 1.0f;
    if (dot < -1.0f) dot = -1.0f;
    float angle = acosf(dot);
    
    // Check for a calibration event (long button press).
    button_event_t btn_event = button_get_event();
    if (btn_event == BUTTON_EVENT_LONG_PRESS) {
        posture_state = POSTURE_STATE_CALIBRATION;
    }
    
    uint32_t now = HAL_GetTick();
    
    switch (posture_state) {
        case POSTURE_STATE_MONITORING:
            if (angle > threshold_angle) {
                // Posture deviates too much—alert the user.
                haptic_feedback_play_waveform(1);  // Use a waveform index for alert.
                haptic_feedback_start();
                alert_start_time = now;
                posture_state = POSTURE_STATE_ALERT;
                led_execute_sequence(LED_SEQ_DOUBLE_BLINK);
            }
            break;
        
        case POSTURE_STATE_ALERT:
            if (angle <= threshold_angle) {
                // The user appears to have corrected their posture.
                if (stable_start_time == 0) {
                    stable_start_time = now;
                } else if ((now - stable_start_time) >= STABLE_DURATION_MS) {
                    // Posture has remained corrected for the required stable period.
                    haptic_feedback_stop();
                    posture_state = POSTURE_STATE_LEARNING;
                }
            } else {
                // If the deviation is still large, reset the stable timer.
                stable_start_time = 0;
            }
            break;
        
        case POSTURE_STATE_LEARNING:
            // Update the reference posture using the corrected measurement.
            update_reference_vector(measured);
            // (Optionally, adjust the threshold here based on correction variability.)
            posture_state = POSTURE_STATE_MONITORING;
            break;
        
        case POSTURE_STATE_CALIBRATION:
            // Manual calibration: set the reference posture to the current measurement.
            for (int i = 0; i < 3; i++) {
                reference_vector[i] = measured[i];
            }
            normalize_vector(reference_vector);
            threshold_angle = DEFAULT_THRESHOLD_ANGLE_RAD;
            // Provide feedback that calibration was successful.
            led_execute_sequence(LED_SEQ_THREE_BLINKS);
            haptic_feedback_play_waveform(2);
            haptic_feedback_start();
            HAL_Delay(500);
            haptic_feedback_stop();
            posture_state = POSTURE_STATE_MONITORING;
            stable_start_time = 0;
            break;
        
        default:
            posture_state = POSTURE_STATE_MONITORING;
            break;
    }
}