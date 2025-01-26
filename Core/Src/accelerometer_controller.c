// TODO:
// use MMA8451Q_REG_STATUS


#include "accelerometer_controller.h"
#include "peripherals.h"
#include "led_controller.h"

static HAL_StatusTypeDef accelerometer_write_reg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1,
                             MMA8451Q_I2C_ADDR,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1,
                             HAL_MAX_DELAY);
}

static HAL_StatusTypeDef accelerometer_read_reg(uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(&hi2c1,
                            MMA8451Q_I2C_ADDR,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            value,
                            1,
                            HAL_MAX_DELAY);
}

static HAL_StatusTypeDef accelerometer_read_regs(uint8_t start_reg, uint8_t *buffer, uint8_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1,
                            MMA8451Q_I2C_ADDR,
                            start_reg,
                            I2C_MEMADD_SIZE_8BIT,
                            buffer,
                            len,
                            HAL_MAX_DELAY);
}

static HAL_StatusTypeDef accel_goto_standby(void)
{
    uint8_t value;
    HAL_StatusTypeDef status = accelerometer_read_reg(MMA8451Q_REG_CTRL_REG1, &value);
    if (status != HAL_OK) return status;

    value &= 0;  // clear ACTIVE bit
    return accelerometer_write_reg(MMA8451Q_REG_CTRL_REG1, value);
}

static HAL_StatusTypeDef accel_goto_active(void)
{
    uint8_t value;
    HAL_StatusTypeDef status = accelerometer_read_reg(MMA8451Q_REG_CTRL_REG1, &value);
    if (status != HAL_OK) return status;

    value |= 1;  // set ACTIVE bit
    return accelerometer_write_reg(MMA8451Q_REG_CTRL_REG1, value);
}

HAL_StatusTypeDef accelerometer_controller_initialize(void)
{
    HAL_StatusTypeDef status;
    uint8_t who_am_i;

    // Read WHO_AM_I
    status = accelerometer_read_reg(MMA8451Q_REG_WHO_AM_I, &who_am_i);
    if (status != HAL_OK) return status;
    if (who_am_i != WHO_AM_I_VALUE) return HAL_ERROR;

    // Put device into Standby
    status = accel_goto_standby();
    if (status != HAL_OK) return status;

    // Configure CTRL_REG1
    status = accelerometer_write_reg(MMA8451Q_REG_CTRL_REG1, CTRL_REG1);
    if (status != HAL_OK) return status;
    
    // Configure CTRL_REG2
    status = accelerometer_write_reg(MMA8451Q_REG_CTRL_REG2, CTRL_REG2);
    if (status != HAL_OK) return status;

    // Configure CTRL_REG3
    status = accelerometer_write_reg(MMA8451Q_REG_CTRL_REG3, CTRL_REG3);
    if (status != HAL_OK) return status;

    // Configure CTRL_REG4
    status = accelerometer_write_reg(MMA8451Q_REG_CTRL_REG4, CTRL_REG4);
    if (status != HAL_OK) return status;

    // Configure CTRL_REG5
    status = accelerometer_write_reg(MMA8451Q_REG_CTRL_REG5, CTRL_REG5);
    if (status != HAL_OK) return status;

    // Configure XYZ_DATA_CFG
    status = accelerometer_write_reg(MMA8451Q_REG_XYZ_DATA_CFG, XYZ_DATA_CFG);
    if (status != HAL_OK) return status;

    // Configure HPF cutoff
    status = accelerometer_write_reg(MMA8451Q_REG_HP_FILTER_CUTOFF, HP_FILTER_CUTOFF);
    if (status != HAL_OK) return status;

    // Configure FF_MT_CFG
    status = accelerometer_write_reg(MMA8451Q_REG_FF_MT_CFG, FF_MT_CFG);
    if (status != HAL_OK) return status;

    // Set motion detection threshold
    status = accelerometer_write_reg(MMA8451Q_REG_FF_MT_THS, ACCEL_MOTION_THRESHOLD_VALUE);
    if (status != HAL_OK) return status;
    
    // Set debounce count
    status = accelerometer_write_reg(MMA8451Q_REG_FF_MT_COUNT, ACCEL_MOTION_DEBOUNCE_COUNT);
    if (status != HAL_OK) return status;

    // Set auto-sleep timeout
    status = accelerometer_write_reg(MMA8451Q_REG_ASLP_COUNT, ASLP_COUNT);
    if (status != HAL_OK) return status;

    // /* 7) Configure Tap detection for X,Y,Z single tap */
    // uint8_t pulse_cfg = (PULSE_CFG_ELE |
    //                      PULSE_CFG_XSPEFE |
    //                      PULSE_CFG_YSPEFE |
    //                      PULSE_CFG_ZSPEFE);
    // status = accelerometer_write_reg(MMA8451Q_REG_PULSE_CFG, pulse_cfg);
    // if (status != HAL_OK) {
    //     return status;
    // }

    // /* Tap thresholds */
    // uint8_t tap_ths = mg_to_threshold_code(ACCEL_TAP_THRESHOLD_MG);
    // status = accelerometer_write_reg(MMA8451Q_REG_PULSE_THSX, tap_ths);
    // if (status != HAL_OK) return status;
    // status = accelerometer_write_reg(MMA8451Q_REG_PULSE_THSY, tap_ths);
    // if (status != HAL_OK) return status;
    // status = accelerometer_write_reg(MMA8451Q_REG_PULSE_THSZ, tap_ths);
    // if (status != HAL_OK) return status;

    // /* Tap timing: TMLT, LTCY, WIND */
    // status = accelerometer_write_reg(MMA8451Q_REG_PULSE_TMLT, ACCEL_TAP_TIME_LIMIT);
    // if (status != HAL_OK) return status;
    // status = accelerometer_write_reg(MMA8451Q_REG_PULSE_LTCY, ACCEL_TAP_LATENCY);
    // if (status != HAL_OK) return status;
    // status = accelerometer_write_reg(MMA8451Q_REG_PULSE_WIND, ACCEL_TAP_WINDOW);
    // if (status != HAL_OK) return status;

    // Put device into Active
    status = accel_goto_active();
    if (status != HAL_OK) return status;

    return HAL_OK;
}

HAL_StatusTypeDef accelerometer_read_mps2(accel_data_t *data)
{
    if (!data) {
        return HAL_ERROR;
    }

    // Read X, Y, Z (6 bytes)
    uint8_t raw[6];
    HAL_StatusTypeDef status = accelerometer_read_regs(MMA8451Q_REG_OUT_X_MSB, raw, 6);
    if (status != HAL_OK) {
        return status;
    }

    // 14-bit sign-extended: (MSB << 8 | LSB) >> 2
    int16_t x = (int16_t)((raw[0] << 8) | raw[1]);  x >>= 2;
    int16_t y = (int16_t)((raw[2] << 8) | raw[3]);  y >>= 2;
    int16_t z = (int16_t)((raw[4] << 8) | raw[5]);  z >>= 2;

    // Convert to g
    float x_g = (float)x / MMA8451Q_SENS_2G_14BIT;
    float y_g = (float)y / MMA8451Q_SENS_2G_14BIT;
    float z_g = (float)z / MMA8451Q_SENS_2G_14BIT;

    /* Convert to m/s^2 */
    data->x_mps2 = x_g * ACCEL_G;
    data->y_mps2 = y_g * ACCEL_G;
    data->z_mps2 = z_g * ACCEL_G;

    return HAL_OK;
}

bool is_accelerometer_in_sleep_mode(void)
{
    uint8_t value;
    HAL_StatusTypeDef status = accelerometer_read_reg(MMA8451Q_REG_SYSMOD, &value);
    if (status != HAL_OK) return false;

    // If bit1 = 0 and bit0 = 0 it is in sleep mode. Only look at bit0 and bit1
    bool bit0 = value & 0x01;
    bool bit1 = value & 0x02;
    return (bit1 && !bit0);
}

bool is_motion_detected(void)
{
    uint8_t value;
    HAL_StatusTypeDef status = accelerometer_read_reg(MMA8451Q_REG_FF_MT_SRC, &value);
    if (status != HAL_OK) return false;

    // if bit7 = 1, motion detected
    bool bit7 = value & 0x80;
    return bit7;
}

// void accelerometer_handle_int1(void)
// {
// }

// void accelerometer_handle_int2(void)
// {
// }