#ifndef POSTURE_CONTROLLER_H
#define POSTURE_CONTROLLER_H

#include "accelerometer_controller.h"
#include "button_controller.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    POSTURE_STATE_MONITORING,
    POSTURE_STATE_ALERT,
    POSTURE_STATE_LEARNING,
    POSTURE_STATE_CALIBRATION
} posture_state_t;

/**
 * @brief Initializes the posture controller.
 *
 * Reads an initial accelerometer sample to set the reference posture.
 */
void posture_controller_initialize(void);

/**
 * @brief Updates the posture controller.
 *
 * Reads the accelerometer, computes the deviation from the reference posture,
 * and, based on the current state, either triggers an alert,
 * waits for a stable correction, updates the reference posture, or performs a calibration.
 */
void posture_controller_update(void);

#ifdef __cplusplus
}
#endif

#endif /* POSTURE_CONTROLLER_H */