#include "sleep_controller.h"
#include "accelerometer_controller.h"
#include "haptic_feedback_controller.h"
#include "stm32f0xx_hal.h"

/* Optional flag to track sleep state */
static volatile uint8_t sleep_mode_active = 0;

void sleep_controller_activate_sleep_mode(void)
{
    // Ensure that the interrupt pin (GPIO_PIN_1 on GPIOA) is low before sleeping.
    // If it is high, wait until it clears.
    clear_accelerometer_interrupts();
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET)
    {
        return;
    }

    haptic_feedback_disable();

    /* Mark that we are about to sleep */
    sleep_mode_active = 1;

    /* Optional: Perform any pre-sleep tasks here, like disabling nonessential peripherals */

    /* Suspend the SysTick interrupt to avoid unwanted wake-ups */
    HAL_SuspendTick();

    /* Enter Sleep mode.
       PWR_MAINREGULATOR_ON: keep the main regulator on.
       PWR_SLEEPENTRY_WFI: use the Wait For Interrupt instruction to enter sleep.
       In this mode, the CPU stops until any enabled interrupt (e.g. from GPIO_PIN_0) occurs.
    */
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

    /* When an interrupt wakes the MCU, execution resumes here */

    /* Resume the SysTick interrupt */
    HAL_ResumeTick();

    haptic_feedback_enable();
    haptic_feedback_controller_initialize();


    /* Mark that sleep is no longer active */
    sleep_mode_active = 0;
}

void sleep_controller_wake_device(void)
{
    /* 
       This function is intended to be called from an interrupt callback (for example, when GPIO_PIN_0 triggers).
       For the simple Sleep mode entered via HAL_PWR_EnterSLEEPMode, the wake-up happens automatically when
       an interrupt occurs. This function is a placeholder where you can add any extra post-wake-up logic,
       such as reinitializing clocks, peripherals, or clearing flags.
    */

    /* Example: if additional reinitialization is needed, add it here. */
}