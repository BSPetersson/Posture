#include "state_machine.h"

device_state_t current_state;

void state_machine_initialize() {
    current_state = DEVICE_STATE_MONITORING_STATE;
}

void state_machine_update() {
    // If button is long pressed, calibrate the accelerometer
    if (button_controller_get_event() == BUTTON_EVENT_LONG_PRESS) {
        current_state = DEVICE_STATE_CALIBRATION_STATE;
    }

    // If the accelerometer detects motion, switch to activity state
    if (accelerometer_controller_is_in_motion()) {
        current_state = DEVICE_STATE_ACTIVITY_STATE;
    }

    // If the accelerometer detects no motion, switch to sleep state
    if (accelerometer_controller_no_motion()) {
        current_state = DEVICE_STATE_SLEEP_STATE;
    }

    switch (current_state) {
        case DEVICE_STATE_MONITORING_STATE:
            handle_monitoring_state();
            break;
        case DEVICE_STATE_BAD_POSTURE_STATE:
            handle_bad_posture_state();
            break;
        case DEVICE_STATE_CALIBRATION_STATE:
            handle_calibration_state();
            break;
        case DEVICE_STATE_ACTIVITY_STATE:
            handle_activity_state();
            break;
        case DEVICE_STATE_SLEEP_STATE:
            handle_sleep_state();
            break;
        case TEST_STATE:
            handle_test_state();
            break;
    }
}

void handle_monitoring_state() {
    if (!posture_controller_is_posture_correct()) {
        current_state = DEVICE_STATE_BAD_POSTURE_STATE;
    }
}

void handle_bad_posture_state() {
    if (posture_controller_handle_bad_posture()) {
        current_state = DEVICE_STATE_MONITORING_STATE;
    }
}

void handle_calibration_state() {
    posture_controller_calibrate();
    current_state = DEVICE_STATE_MONITORING_STATE;
}

void handle_activity_state() {
    if (!accelerometer_controller_is_in_motion()) {
        current_state = DEVICE_STATE_MONITORING_STATE;
    }
}

void handle_sleep_state() {
    sleep_controller_activate_sleep_mode();

    // When this function returns, the device has woken up
    current_state = DEVICE_STATE_MONITORING_STATE;
}

void handle_test_state() {
    // Do nothing
}