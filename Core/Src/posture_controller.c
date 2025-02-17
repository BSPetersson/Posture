#include "posture_controller.h"
#include "haptic_feedback_controller.h"
#include "button_controller.h"
#include "led_controller.h"
#include "main.h"
#include <math.h>

// Constants
#define DEFAULT_THRESHOLD_ANGLE_RAD (20.0f * (M_PI / 180.0f))  // 15 degrees (in radians)
#define EMA_ALPHA                   0.5f    // Weight for exponential moving average update

// Internal state variables
static float measured_vector[3];
static float reference_vector[3];
static float threshold_angle = DEFAULT_THRESHOLD_ANGLE_RAD;
static float current_angle = 0.0f;
static bool is_posture_correct = true;
uint32_t last_bad_posture_time = 0;
uint32_t last_good_posture_time = 0;

static void normalize_vector(float v[3]) {
    float mag = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (mag > 0.0f) {
        v[0] /= mag;
        v[1] /= mag;
        v[2] /= mag;
    }
}

static float dot_product(const float a[3], const float b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static void update_reference_vector(void) {
    for (int i = 0; i < 3; i++) {
        reference_vector[i] = (1.0f - EMA_ALPHA) * reference_vector[i] + EMA_ALPHA * measured_vector[i];
    }
    normalize_vector(reference_vector);
}

static float get_angle(void) {
    accel_data_t data = accelerometer_controller_get_latest_data();
    
    measured_vector[0] = data.x_mps2;
    measured_vector[1] = data.y_mps2;
    measured_vector[2] = data.z_mps2;
    normalize_vector(measured_vector);
    
    float dot = dot_product(reference_vector, measured_vector);
    if (dot > 1.0f)  dot = 1.0f;
    if (dot < -1.0f) dot = -1.0f;
    return acosf(dot);
}

void posture_controller_initialize(void) {
    threshold_angle = DEFAULT_THRESHOLD_ANGLE_RAD;
    
    accel_data_t data = accelerometer_controller_get_latest_data();
    reference_vector[0] = data.x_mps2;
    reference_vector[1] = data.y_mps2;
    reference_vector[2] = data.z_mps2;
    normalize_vector(reference_vector);
}

void posture_controller_update(void) {
    uint32_t now = HAL_GetTick();
    current_angle = get_angle();

    if (current_angle > threshold_angle) {
        last_bad_posture_time = now;
    } else {
        last_good_posture_time = now;
    }

    if (last_good_posture_time > last_bad_posture_time && (last_good_posture_time - last_bad_posture_time) > 1000 && !is_posture_correct) {
        is_posture_correct = true;
        update_reference_vector();
    }

    if (last_bad_posture_time > last_good_posture_time && (last_bad_posture_time - last_good_posture_time) > 1000 && is_posture_correct) {
        is_posture_correct = false;
    }
}

bool posture_controller_is_posture_correct(void) {
    haptic_feedback_disable();
    led_off();
    return is_posture_correct;
}

bool posture_controller_handle_bad_posture(void) {
    // Trigger hapic feedback constantly
    haptic_feedback_play_waveform(0x01);
    haptic_feedback_start();
    led_on(100);
    return is_posture_correct;
}

void posture_controller_calibrate(void) {
    accel_data_t data = accelerometer_controller_get_latest_data();
    reference_vector[0] = data.x_mps2;
    reference_vector[1] = data.y_mps2;
    reference_vector[2] = data.z_mps2;
    normalize_vector(reference_vector);
}