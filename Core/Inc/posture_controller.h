#ifndef POSTURE_CONTROLLER_H
#define POSTURE_CONTROLLER_H

#include "accelerometer_controller.h"
#include "led_controller.h"
#include "haptic_feedback_controller.h"
#include "button_controller.h"
#include "sleep_controller.h"
#include <stdbool.h>

// Function declarations
void posture_controller_initialize(void);
void posture_controller_update(void);
void react_to_bad_posture(void);
void react_to_initial_still_bad_posture(void);
void react_to_posture_correct(void);
void start_calibration_procedure(void);
void react_to_device_off(void);
void react_to_device_on(void);
void reset_good_posture_vector(void);
void react_to_calibration(void);
void update_good_posture_vector();
void update_good_posture_history(float vector[3]);
void reset_good_posture_history(void);

#endif /* POSTURE_CONTROLLER_H */