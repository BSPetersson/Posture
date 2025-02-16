#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "haptic_feedback_controller.h"
#include "led_controller.h"
#include "button_controller.h"
#include "accelerometer_controller.h"
#include "posture_controller.h"
#include "sleep_controller.h"

typedef enum {
    DEVICE_STATE_MONITORING_STATE,
    DEVICE_STATE_BAD_POSTURE_STATE,
    DEVICE_STATE_CALIBRATION_STATE,
    DEVICE_STATE_ACTIVITY_STATE,
    DEVICE_STATE_SLEEP_STATE,
    TEST_STATE
} device_state_t;

void state_machine_initialize();
void state_machine_update();

void handle_monitoring_state();
void handle_bad_posture_state();
void handle_calibration_state();
void handle_activity_state();
void handle_sleep_state();
void handle_test_state();

#endif // STATE_MACHINE_H
