#ifndef POSTURE_CONTROLLER_H
#define POSTURE_CONTROLLER_H

#include "accelerometer_controller.h"
#include "button_controller.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the posture controller.
 */
void posture_controller_initialize(void);

/**
 * @brief Non-blocking update function.
 *
 * This function must be called continuously (e.g. in your main loop) to update
 * the posture state.
 */
void posture_controller_update(void);

/**
 * @brief Returns true if the current posture is considered correct.
 *
 * Posture is considered incorrect if the angular deviation has exceeded the threshold
 * continuously for a set duration.
 */
bool posture_controller_is_posture_correct(void);

/**
 * @brief Returns true if a sustained bad posture has been detected and handled.
 *
 * When this condition is met the reference posture vector is updated and haptic feedback is triggered.
 */
bool posture_controller_handle_bad_posture(void);

/**
 * @brief Calibrates the posture controller by resetting the reference vector to the current reading.
 * @return true if calibration was successful.
 */
void posture_controller_calibrate(void);

#ifdef __cplusplus
}
#endif

#endif /* POSTURE_CONTROLLER_H */