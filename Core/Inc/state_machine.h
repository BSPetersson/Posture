#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

// #include "power_manager.h"
#include "haptic_feedback_controller.h"
#include "led_controller.h"
// #include "posture_monitor.h"
// #include "calibration_manager.h"
// #include "adaptive_learning_engine.h"
// #include "movement_detector.h"

typedef enum {
    DEVICE_STATE_SLEEP_MODE,
    DEVICE_STATE_ACTIVE_MODE,
    DEVICE_STATE_CALIBRATION_MODE,
    DEVICE_STATE_ACTIVITY_MODE,
    DEVICE_STATE_BAD_POSTURE_STATE,
    DEVICE_STATE_GOOD_POSTURE_STATE,
    TEST_STATE
} device_state_t;

void state_machine_initialize();

void state_machine_update();

void handle_sleep_mode();
void handle_active_mode();
void handle_calibration_mode();
void handle_activity_mode();
void handle_bad_posture_state();
void handle_good_posture_state();
void transition_to(device_state_t new_state);

void handle_test_accelerometer();
void handle_test_state();

#endif // STATE_MACHINE_H
