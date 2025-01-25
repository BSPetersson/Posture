#ifndef HAPTIC_FEEDBACK_CONTROLLER_H
#define HAPTIC_FEEDBACK_CONTROLLER_H

#include "stm32f0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// -----------------------------
// Register Addresses
// -----------------------------

#define DRV2605L_REG_FEEDBACK_CONTROL  0x1A
#define DRV2605L_REG_RATED_VOLTAGE     0x16
#define DRV2605L_REG_MODE               0x01
#define DRV2605L_REG_WAVESEQ1           0x04
#define DRV2605L_REG_GO                 0x0C
#define DRV2605L_REG_RTP                0x02
#define DRV2605L_REG_LIBRARY            0x03
#define DRV2605L_REG_STATUS             0x00
#define DRV2605L_REG_OD_CLAMP           0x17
#define DRV2605L_REG_CONTROL1           0x1B
#define DRV2605L_REG_CONTROL2           0x1C
#define DRV2605L_REG_CONTROL3           0x1D
#define DRV2605L_REG_CONTROL4           0x1E
#define DRV2605L_REG_CONTROL5           0x1F
#define DRV2605L_REG_OPEN_LOOP_PERIOD   0x20

// -----------------------------
// Mode Values
// -----------------------------

#define DRV2605L_MODE_INTERNAL_TRIGGER          0x00
#define DRV2605L_MODE_EXTERNAL_TRIGGER_EDGE      0x01
#define DRV2605L_MODE_EXTERNAL_TRIGGER_LEVEL     0x02
#define DRV2605L_MODE_PWM_ANALOG                 0x03
#define DRV2605L_MODE_AUDIO_TO_VIBE              0x04
#define DRV2605L_MODE_REAL_TIME_PLAYBACK         0x05
#define DRV2605L_MODE_DIAGNOSTICS                0x06
#define DRV2605L_MODE_CALIBRATION                0x07

// -----------------------------
// I2C Address
// -----------------------------

#define DRV2605L_I2C_ADDR 0x5A

// -----------------------------
// Pin Definitions for Haptic Driver
// -----------------------------

#define HAPTIC_EN_PIN  GPIO_PIN_13
#define HAPTIC_EN_PORT GPIOB

// -----------------------------
// Enum for Actuator Types
// -----------------------------

typedef enum {
    ERM = 0,
    LRA = 1
} actuator_type_t;

// -----------------------------
// Function Prototypes
// -----------------------------

// Initialization and Control
void haptic_feedback_controller_initialize(void);
void haptic_feedback_enable(void);
void haptic_feedback_disable(void);
bool haptic_feedback_run_auto_calibration_procedure(void);

// Calibration and Diagnostics
HAL_StatusTypeDef haptic_feedback_calibrate(void);
HAL_StatusTypeDef haptic_feedback_diagnostics(void);
HAL_StatusTypeDef haptic_feedback_get_calibration_diagnostic_result(uint8_t *diagnostic_result);
HAL_StatusTypeDef haptic_feedback_init_for_wideband_LRA(void);

// Configuration Setters
HAL_StatusTypeDef haptic_feedback_set_actuator_type(actuator_type_t actuator_type);
HAL_StatusTypeDef haptic_feedback_set_brake_factor(uint8_t brake_factor);
HAL_StatusTypeDef haptic_feedback_set_loop_gain(uint8_t loop_gain);
HAL_StatusTypeDef haptic_feedback_set_rated_voltage(uint8_t rated_voltage_tenths);
HAL_StatusTypeDef haptic_feedback_set_od_clamp(uint8_t od_clamp);
HAL_StatusTypeDef haptic_feedback_set_auto_calibration_time(uint8_t auto_calibration_time);
HAL_StatusTypeDef haptic_feedback_set_drive_time(uint8_t drive_time);
HAL_StatusTypeDef haptic_feedback_set_sample_time(uint8_t sample_time);
HAL_StatusTypeDef haptic_feedback_set_blanking_time(uint8_t blanking_time);
HAL_StatusTypeDef haptic_feedback_set_idiss_time(uint8_t idiss_time);
HAL_StatusTypeDef haptic_feedback_set_zc_det_time(uint8_t zc_det_time);
HAL_StatusTypeDef haptic_feedback_set_open_loop(bool open_loop);
HAL_StatusTypeDef haptic_feedback_set_ol_lra_period(uint8_t frequency_hz);

// Operation Control
HAL_StatusTypeDef haptic_feedback_play_waveform(uint8_t waveform);
HAL_StatusTypeDef haptic_feedback_set_mode(uint8_t mode);
HAL_StatusTypeDef haptic_feedback_set_library(uint8_t library);
HAL_StatusTypeDef haptic_feedback_start(void);
HAL_StatusTypeDef haptic_feedback_stop(void);
HAL_StatusTypeDef haptic_feedback_rtp_mode(uint8_t amplitude);
void haptic_feedback_reset(void);

#endif // HAPTIC_FEEDBACK_CONTROLLER_H