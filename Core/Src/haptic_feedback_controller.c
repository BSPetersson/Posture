#include "haptic_feedback_controller.h"
#include "peripherals.h"

/**
 * @brief Modifies specific bits in a given register via I2C.
 *
 * This function performs a read-modify-write operation on a specified register.
 *
 * @param reg_address The address of the register to modify.
 * @param mask        The bitmask indicating which bits to modify.
 * @param value       The new value to set for the specified bits.
 * @return HAL_StatusTypeDef Status of the I2C operations.
 */
static HAL_StatusTypeDef haptic_feedback_modify_register(uint8_t reg_address, uint8_t mask, uint8_t value) {
    HAL_StatusTypeDef status;
    uint8_t reg_value;

    // Transmit the register address
    status = HAL_I2C_Master_Transmit(&hi2c1, (DRV2605L_I2C_ADDR << 1), &reg_address, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    // Receive the current register value
    status = HAL_I2C_Master_Receive(&hi2c1, (DRV2605L_I2C_ADDR << 1), &reg_value, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    // Modify the register value
    reg_value = (reg_value & ~mask) | (value & mask);

    // Transmit the modified register value
    uint8_t data[2] = {reg_address, reg_value};
    return HAL_I2C_Master_Transmit(&hi2c1, (DRV2605L_I2C_ADDR << 1), data, 2, HAL_MAX_DELAY);
}

/**
 * @brief Initializes the haptic feedback controller.
 *
 * This function starts the auto-calibration procedure.
 */
void haptic_feedback_controller_initialize(void) {
    bool success = false;

    while (!success) {
        HAL_StatusTypeDef status = haptic_feedback_init_for_wideband_LRA();
        if (status == HAL_OK) {
            success = true;
        }
        HAL_Delay(100);
    }
}

/**
 * @brief Runs the auto-calibration procedure for the haptic feedback controller.
 */
bool haptic_feedback_run_auto_calibration_procedure(void) {
    HAL_StatusTypeDef status;

    haptic_feedback_enable();
    haptic_feedback_set_mode(DRV2605L_MODE_CALIBRATION);

    // Populate the input parameters required by the auto-calibration engine
    status  = haptic_feedback_set_actuator_type(LRA);
    status |= haptic_feedback_set_brake_factor(2);
    status |= haptic_feedback_set_loop_gain(2);
    status |= haptic_feedback_set_rated_voltage(105);
    status |= haptic_feedback_set_od_clamp(126);
    status |= haptic_feedback_set_auto_calibration_time(3);
    status |= haptic_feedback_set_drive_time(24);
    status |= haptic_feedback_set_sample_time(3);
    status |= haptic_feedback_set_blanking_time(1);
    status |= haptic_feedback_set_idiss_time(1);
    status |= haptic_feedback_set_zc_det_time(0);

    haptic_feedback_start();

    HAL_Delay(5000); // Wait for the calibration to complete

    uint8_t diagnostic_result;
    status |= haptic_feedback_get_calibration_diagnostic_result(&diagnostic_result);

    // checkout if bit 3 is 0 or 1. if 1, calibration failed and we should retry
    while ((diagnostic_result & 0x08) != 0) {
        return false;
    }

    // Pause for a moment to allow the calibration to complete
    HAL_Delay(5000);

    return true;
}

/**
 * @brief Initializes the haptic feedback controller for a wideband LRA.
 *
 * This function configures the DRV2605L driver for a wideband LRA actuator.
 *
 * @return HAL_StatusTypeDef Status of the I2C operations.
 */
HAL_StatusTypeDef haptic_feedback_init_for_wideband_LRA(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    // 1) Enable the DRV2605L driver pin
    haptic_feedback_enable();

    // 2) Put the device in Internal Trigger Mode (0x00). 
    //    This also clears STANDBY (bit 6 in MODE register).
    status = haptic_feedback_set_mode(DRV2605L_MODE_INTERNAL_TRIGGER);
    if (status != HAL_OK) {
        return status;
    }

    // 3) Select LRA in the FEEDBACK_CONTROL register (bit 7).
    //    This ensures the device is configured for LRA rather than ERM.
    status = haptic_feedback_set_actuator_type(LRA);
    if (status != HAL_OK) {
        return status;
    }

    // 4) Write directly to CONTROL3 (Register 0x1D) to enable LRA open-loop.
    status = haptic_feedback_set_open_loop(true);
    if (status != HAL_OK) {
        return status;
    }

    // 5) Set OL_LRA_PERIOD (Register 0x20) for ~170 Hz
    status = haptic_feedback_set_ol_lra_period(170);
    if (status != HAL_OK) {
        return status;
    }

    // 6) (Optional) Set OD_CLAMP so the driver does not exceed ~2.5–3 V beyond rated voltage
    //    in open-loop. Adjust as needed.
    status = haptic_feedback_set_od_clamp(126);
    if (status != HAL_OK) {
        return status;
    }

    return status;
}

/**
 * @brief Enables the haptic driver.
 */
inline void haptic_feedback_enable(void) {
    HAL_GPIO_WritePin(HAPTIC_EN_PORT, HAPTIC_EN_PIN, GPIO_PIN_SET);
}

/**
 * @brief Disables the haptic driver.
 */
inline void haptic_feedback_disable(void) {
    HAL_GPIO_WritePin(HAPTIC_EN_PORT, HAPTIC_EN_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Configures the actuator type (ERM or LRA).
 *
 * @param actuator_type The type of actuator to set.
 * @return HAL_StatusTypeDef Status of the I2C operation.
 */
HAL_StatusTypeDef haptic_feedback_set_actuator_type(actuator_type_t actuator_type) {
    uint8_t mask = 0x80; // Bit 7
    uint8_t value = (actuator_type == LRA) ? 0x80 : 0x00;
    return haptic_feedback_modify_register(DRV2605L_REG_FEEDBACK_CONTROL, mask, value);
}

/**
 * @brief Sets the brake factor.
 *
 * @param brake_factor The brake factor value (should fit in 3 bits).
 * @return HAL_StatusTypeDef Status of the I2C operation.
 */
HAL_StatusTypeDef haptic_feedback_set_brake_factor(uint8_t brake_factor) {
    uint8_t mask = 0x70; // Bits 4,5,6
    uint8_t value = (brake_factor << 4) & mask;
    return haptic_feedback_modify_register(DRV2605L_REG_FEEDBACK_CONTROL, mask, value);
}

/**
 * @brief Sets the loop gain.
 *
 * @param loop_gain The loop gain value (should fit in 2 bits).
 * @return HAL_StatusTypeDef Status of the I2C operation.
 */
HAL_StatusTypeDef haptic_feedback_set_loop_gain(uint8_t loop_gain) {
    uint8_t mask = 0x0C; // Bits 2,3
    uint8_t value = (loop_gain << 2) & mask;
    return haptic_feedback_modify_register(DRV2605L_REG_FEEDBACK_CONTROL, mask, value);
}

/**
 * @brief Sets the rated voltage.
 *
 * @param rated_voltage_tenths Rated voltage
 * @return HAL_StatusTypeDef Status of the I2C operation.
 */
HAL_StatusTypeDef haptic_feedback_set_rated_voltage(uint8_t rated_voltage) {
    uint8_t data[2] = {DRV2605L_REG_RATED_VOLTAGE, rated_voltage};
    return HAL_I2C_Master_Transmit(&hi2c1, (DRV2605L_I2C_ADDR << 1), data, 2, HAL_MAX_DELAY);
}

/**
 * @brief Sets the overdrive clamp (OD_CLAMP) value.
 *
 * @param od_clamp The OD_CLAMP value.
 * @return HAL_StatusTypeDef Status of the I2C operation.
 */
HAL_StatusTypeDef haptic_feedback_set_od_clamp(uint8_t od_clamp) {
    uint8_t data[2] = {DRV2605L_REG_OD_CLAMP, od_clamp};
    return HAL_I2C_Master_Transmit(&hi2c1, (DRV2605L_I2C_ADDR << 1), data, 2, HAL_MAX_DELAY);
}

/**
 * @brief Sets the auto calibration time.
 *
 * @param auto_calibration_time The auto calibration time value (should fit in 2 bits).
 * @return HAL_StatusTypeDef Status of the I2C operation.
 */
HAL_StatusTypeDef haptic_feedback_set_auto_calibration_time(uint8_t auto_calibration_time) {
    uint8_t mask = 0x30; // Bits 4,5 in CONTROL4
    uint8_t value = (auto_calibration_time << 4) & mask;
    return haptic_feedback_modify_register(DRV2605L_REG_CONTROL4, mask, value);
}

/**
 * @brief Sets the drive time.
 *
 * @param drive_time The drive time value (should fit in 5 bits).
 * @return HAL_StatusTypeDef Status of the I2C operation.
 */
HAL_StatusTypeDef haptic_feedback_set_drive_time(uint8_t drive_time) {
    uint8_t mask = 0x1F; // Bits 0-4 in CONTROL1
    uint8_t value = drive_time & mask;
    return haptic_feedback_modify_register(DRV2605L_REG_CONTROL1, mask, value);
}

/**
 * @brief Sets the sample time.
 *
 * @param sample_time The sample time value (should fit in 2 bits).
 * @return HAL_StatusTypeDef Status of the I2C operation.
 */
HAL_StatusTypeDef haptic_feedback_set_sample_time(uint8_t sample_time) {
    uint8_t mask = 0x30; // Bits 4,5 in CONTROL2
    uint8_t value = (sample_time << 4) & mask;
    return haptic_feedback_modify_register(DRV2605L_REG_CONTROL2, mask, value);
}

/**
 * @brief Sets the blanking time.
 *
 * @param blanking_time The blanking time value (0-15).
 * @return HAL_StatusTypeDef Status of the I2C operation.
 */
HAL_StatusTypeDef haptic_feedback_set_blanking_time(uint8_t blanking_time) {
    if (blanking_time > 15) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    // Set lower 2 bits in CONTROL2 (bits 2,3)
    status = haptic_feedback_modify_register(DRV2605L_REG_CONTROL2, 0x0C, (blanking_time & 0x03) << 2);
    if (status != HAL_OK) {
        return status;
    }

    // Set upper 2 bits in CONTROL5 (bits 2,3)
    status = haptic_feedback_modify_register(DRV2605L_REG_CONTROL5, 0x0C, ((blanking_time & 0x0C) >> 2) << 2);
    return status;
}

/**
 * @brief Sets the IDISS time.
 *
 * @param idiss_time The IDISS time value (0-15).
 * @return HAL_StatusTypeDef Status of the I2C operation.
 */
HAL_StatusTypeDef haptic_feedback_set_idiss_time(uint8_t idiss_time) {
    if (idiss_time > 15) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    // Set lower 2 bits in CONTROL2 (bits 0,1)
    status = haptic_feedback_modify_register(DRV2605L_REG_CONTROL2, 0x03, idiss_time & 0x03);
    if (status != HAL_OK) {
        return status;
    }

    // Set upper 2 bits in CONTROL5 (bits 0,1)
    status = haptic_feedback_modify_register(DRV2605L_REG_CONTROL5, 0x03, ((idiss_time & 0x0C) >> 2));
    return status;
}

/**
 * @brief Sets the zero-cross detection time.
 *
 * @param zc_det_time The zero-cross detection time value (should fit in 2 bits).
 * @return HAL_StatusTypeDef Status of the I2C operation.
 */
HAL_StatusTypeDef haptic_feedback_set_zc_det_time(uint8_t zc_det_time) {
    uint8_t mask = 0xC0; // Bits 6,7 in CONTROL4
    uint8_t value = (zc_det_time << 6) & mask;
    return haptic_feedback_modify_register(DRV2605L_REG_CONTROL4, mask, value);
}

/**
 * @brief Sets the open-loop mode.
 *
 * @param open_loop Whether to enable open-loop mode.
 * @return HAL_StatusTypeDef Status of the I2C operation.
 */
HAL_StatusTypeDef haptic_feedback_set_open_loop(bool open_loop) {
    uint8_t mask = 0x01; // Bit 0 in CONTROL3
    uint8_t value = open_loop ? 0x01 : 0x00;
    return haptic_feedback_modify_register(DRV2605L_REG_CONTROL3, mask, value);
}

/**
 * @brief Sets the open-loop LRA period.
 *
 * @param frequency_hz The frequency in Hz to set the period for.
 * @return HAL_StatusTypeDef Status of the I2C operation.
 */
HAL_StatusTypeDef haptic_feedback_set_ol_lra_period(uint8_t frequency_hz) {
    const double factor = 98.46e-6;

    // Calculate the period based on the frequency
    unsigned int ol_lra_period = (unsigned int)(1.0 / (frequency_hz * factor));

    uint8_t mask = 0x7F; // Bits 0-6 in OL_LRA_PERIOD
    uint8_t value = ol_lra_period & mask;
    return haptic_feedback_modify_register(DRV2605L_REG_OPEN_LOOP_PERIOD, mask, value);
}

/**
 * @brief Retrieves the calibration diagnostic result.
 *
 * @param diagnostic_result Pointer to store the diagnostic result.
 * @return HAL_StatusTypeDef Status of the I2C operations.
 */
HAL_StatusTypeDef haptic_feedback_get_calibration_diagnostic_result(uint8_t *diagnostic_result) {
    haptic_feedback_set_mode(DRV2605L_MODE_DIAGNOSTICS);

    uint8_t reg_address = DRV2605L_REG_STATUS;
    HAL_StatusTypeDef status;

    // Transmit the STATUS register address
    status = HAL_I2C_Master_Transmit(&hi2c1, (DRV2605L_I2C_ADDR << 1), &reg_address, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    // Receive the diagnostic result
    status = HAL_I2C_Master_Receive(&hi2c1, (DRV2605L_I2C_ADDR << 1), diagnostic_result, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    return HAL_OK;
}

/**
 * @brief Plays a specific waveform from the library.
 *
 * @param waveform The waveform index to play.
 * @return HAL_StatusTypeDef Status of the I2C operation.
 */
HAL_StatusTypeDef haptic_feedback_play_waveform(uint8_t waveform) {
    uint8_t data[2] = {DRV2605L_REG_WAVESEQ1, waveform};
    return HAL_I2C_Master_Transmit(&hi2c1, (DRV2605L_I2C_ADDR << 1), data, 2, HAL_MAX_DELAY);
}

/**
 * @brief Sets the operation mode of the haptic driver.
 *
 * @param mode The mode to set.
 * @return HAL_StatusTypeDef Status of the I2C operation.
 */
HAL_StatusTypeDef haptic_feedback_set_mode(uint8_t mode) {
    uint8_t data[2] = {DRV2605L_REG_MODE, mode};
    return HAL_I2C_Master_Transmit(&hi2c1, (DRV2605L_I2C_ADDR << 1), data, 2, HAL_MAX_DELAY);
}

/**
 * @brief Sets the haptic effect library.
 *
 * @param library The library index to set.
 * @return HAL_StatusTypeDef Status of the I2C operation.
 */
HAL_StatusTypeDef haptic_feedback_set_library(uint8_t library) {
    uint8_t data[2] = {DRV2605L_REG_LIBRARY, library};
    return HAL_I2C_Master_Transmit(&hi2c1, (DRV2605L_I2C_ADDR << 1), data, 2, HAL_MAX_DELAY);
}

/**
 * @brief Starts the haptic driver to play the selected waveform.
 *
 * @return HAL_StatusTypeDef Status of the I2C operation.
 */
HAL_StatusTypeDef haptic_feedback_start(void) {
    uint8_t data[2] = {DRV2605L_REG_GO, 0x01};
    return HAL_I2C_Master_Transmit(&hi2c1, (DRV2605L_I2C_ADDR << 1), data, 2, HAL_MAX_DELAY);
}

/**
 * @brief Stops the haptic driver.
 *
 * @return HAL_StatusTypeDef Status of the I2C operation.
 */
HAL_StatusTypeDef haptic_feedback_stop(void) {
    uint8_t data[2] = {DRV2605L_REG_GO, 0x00};
    return HAL_I2C_Master_Transmit(&hi2c1, (DRV2605L_I2C_ADDR << 1), data, 2, HAL_MAX_DELAY);
}

/**
 * @brief Enables Real-Time Playback (RTP) mode with a given amplitude.
 *
 * @param amplitude The amplitude level for RTP.
 * @return HAL_StatusTypeDef Status of the I2C operations.
 */
HAL_StatusTypeDef haptic_feedback_rtp_mode(uint8_t amplitude) {
    HAL_StatusTypeDef status;

    status = haptic_feedback_set_mode(DRV2605L_MODE_REAL_TIME_PLAYBACK);
    if (status != HAL_OK) {
        return status;
    }

    uint8_t data[2] = {DRV2605L_REG_RTP, amplitude};
    return HAL_I2C_Master_Transmit(&hi2c1, (DRV2605L_I2C_ADDR << 1), data, 2, HAL_MAX_DELAY);
}

/**
 * @brief Runs the calibration routine.
 *
 * @return HAL_StatusTypeDef Status of the I2C operation.
 */
HAL_StatusTypeDef haptic_feedback_calibrate(void) {
    return haptic_feedback_set_mode(DRV2605L_MODE_CALIBRATION);
}

/**
 * @brief Runs the diagnostics routine.
 *
 * @return HAL_StatusTypeDef Status of the I2C operation.
 */
HAL_StatusTypeDef haptic_feedback_diagnostics(void) {
    return haptic_feedback_set_mode(DRV2605L_MODE_DIAGNOSTICS);
}

/**
 * @brief Resets the haptic feedback controller.
 */
void haptic_feedback_reset(void) {
    uint8_t data[2] = {DRV2605L_REG_MODE, 0x80}; // 0x80 is the reset command
    HAL_I2C_Master_Transmit(&hi2c1, (DRV2605L_I2C_ADDR << 1), data, 2, HAL_MAX_DELAY);
}
