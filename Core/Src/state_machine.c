#include "state_machine.h"

device_state_t current_state;

void state_machine_initialize() {
    current_state = TEST_STATE;
}

void state_machine_update() {
    switch (current_state) {
        case DEVICE_STATE_SLEEP_MODE:
            // handle_sleep_mode(sm);
            break;
        case DEVICE_STATE_ACTIVE_MODE:
            // handle_active_mode(sm);
            break;
        case DEVICE_STATE_CALIBRATION_MODE:
            // handle_calibration_mode(sm);
            break;
        case DEVICE_STATE_ACTIVITY_MODE:
            // handle_activity_mode(sm);
            break;
        case DEVICE_STATE_BAD_POSTURE_STATE:
            // handle_bad_posture_state(sm);
            break;
        case DEVICE_STATE_GOOD_POSTURE_STATE:
            // handle_good_posture_state(sm);
            break;
        case TEST_STATE:
            handle_test_state();
            break;
    }
}


void handle_test_state() {
    // Enable the haptic feedback controller
    haptic_feedback_enable();

    led_execute_sequence(LED_SEQ_THREE_BLINKS);

    // Wait a moment to ensure the controller is ready
    HAL_Delay(10000);

    led_execute_sequence(LED_SEQ_DOUBLE_BLINK);

    haptic_feedback_set_actuator_type(LRA);

    // Test setting the mode to internal trigger mode
    int8_t HAL_STATUS = haptic_feedback_set_mode(DRV2605L_MODE_INTERNAL_TRIGGER);

    // Play a basic waveform from the library
    if (haptic_feedback_play_waveform(1) == HAL_OK) { // Play waveform index 1
        haptic_feedback_start();
        HAL_Delay(1000); // Allow the waveform to play for 1 second
        haptic_feedback_stop();
    }

    led_execute_sequence(LED_SEQ_FADE_IN_OUT);

    for (int i = 0; i <= 123; i++) {
        led_on(i);
        // Play a different waveform
        if (haptic_feedback_play_waveform(i) == HAL_OK) { // Play waveform index 2
            haptic_feedback_start();
            HAL_Delay(1000); // Play for 1 second
            haptic_feedback_stop();
        }

        led_off();
        HAL_Delay(1000); // Wait between waveforms
    }

    // Test real-time playback (RTP) mode with amplitude control
    if (haptic_feedback_rtp_mode(128) == HAL_OK) { // Set amplitude to mid-range
        HAL_Delay(1000); // Play RTP mode for 1 second
        haptic_feedback_stop();
    }

    // Disable the haptic feedback controller after tests are completed
    haptic_feedback_disable();

    // Add a small delay before transitioning out of the test state
    HAL_Delay(500);
}
