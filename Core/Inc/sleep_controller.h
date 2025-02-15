#ifndef SLEEP_CONTROLLER_H
#define SLEEP_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Enters sleep mode.
 *
 * This function suspends the SysTick, enters Sleep mode (CPU halted until an interrupt occurs),
 * and then resumes the SysTick timer upon wake-up.
 *
 * Use this function when you want to reduce power consumption while still keeping peripherals running.
 */
void sleep_controller_activate_sleep_mode(void);

/**
 * @brief Handles additional wake-up actions.
 *
 * This function is intended to be called from an interrupt (e.g. from GPIO_PIN_0 in HAL_GPIO_EXTI_Callback)
 * to perform any post-wake-up processing (like reinitializing peripherals or clearing flags).
 *
 * Note: For a simple Sleep mode, the wake-up is automatic (via the WFI instruction). This function serves as
 * a placeholder for any additional tasks that must run after wake-up.
 */
void sleep_controller_wake_device(void);

#ifdef __cplusplus
}
#endif

#endif /* SLEEP_CONTROLLER_H */