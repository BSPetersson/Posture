#ifndef POSTURE_CONTROLLER_H
#define POSTURE_CONTROLLER_H

#include "accelerometer_controller.h"
#include "led_controller.h"
#include "haptic_feedback_controller.h"
#include "button_controller.h"
#include <stdbool.h>

// Function declarations
void posture_controller_update(void);
void handle_bad_posture(void);
void notify_posture_correct(void);
void start_calibration_procedure(void);
void reset_good_posture_vector(void);
void update_good_posture_vector(float vector[3]);
void update_good_posture_history(float vector[3]);

#endif /* POSTURE_CONTROLLER_H */