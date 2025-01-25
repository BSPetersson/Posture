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

    value &= ~CTRL_REG1_ACTIVE;  // clear ACTIVE bit
    return accelerometer_write_reg(MMA8451Q_REG_CTRL_REG1, value);
}

static HAL_StatusTypeDef accel_goto_active(void)
{
    uint8_t value;
    HAL_StatusTypeDef status = accelerometer_read_reg(MMA8451Q_REG_CTRL_REG1, &value);
    if (status != HAL_OK) return status;

    value |= CTRL_REG1_ACTIVE;  // set ACTIVE bit
    return accelerometer_write_reg(MMA8451Q_REG_CTRL_REG1, value);
}

static uint8_t mg_to_threshold_code(uint16_t mg)
{
    float counts = (float)mg / 63.0f;  // 63 mg per LSB
    if (counts > 127.0f) counts = 127.0f;
    return (uint8_t)(counts + 0.5f);
}

HAL_StatusTypeDef accelerometer_controller_initialize(void)
{
    HAL_StatusTypeDef status;
    uint8_t who_am_i;

    // Read WHO_AM_I
    status = accelerometer_read_reg(MMA8451Q_REG_WHO_AM_I, &who_am_i);
    if (status != HAL_OK) {
        return status;
    }
    if (who_am_i != WHO_AM_I_VALUE) {
        return HAL_ERROR; // Device not recognized
    }

    // Put device into Standby
    status = accel_goto_standby();
    if (status != HAL_OK) {
        return status;
    }

    // Configure g range and high-pass filter
    status = accelerometer_write_reg(MMA8451Q_REG_XYZ_DATA_CFG, XYZ_DATA_CFG_HP_OFF_FS_2G);
    if (status != HAL_OK) {
        return status;
    }

    // Configure HPF cutoff
    status = accelerometer_write_reg(MMA8451Q_REG_HP_FILTER_CUTOFF, HP_FILTER_CUTOFF);
    if (status != HAL_OK) {
        return status;
    }

    // /* 5) Configure FF_MT for motion detection */
    // uint8_t ff_mt_cfg = (FF_MT_CFG_ELE | FF_MT_CFG_OAE |
    //                      FF_MT_CFG_XEFE | FF_MT_CFG_YEFE | FF_MT_CFG_ZEFE);
    // status = accelerometer_write_reg(MMA8451Q_REG_FF_MT_CFG, ff_mt_cfg);
    // if (status != HAL_OK) {
    //     return status;
    // }

    // /* 6) Set motion threshold (mg => code) and debounce */
    // uint8_t ths_code = mg_to_threshold_code(ACCEL_MOTION_THRESHOLD_MG);
    // status = accelerometer_write_reg(MMA8451Q_REG_FF_MT_THS, ths_code);
    // if (status != HAL_OK) return status;

    // status = accelerometer_write_reg(MMA8451Q_REG_FF_MT_COUNT, ACCEL_MOTION_DEBOUNCE_COUNT);
    // if (status != HAL_OK) return status;

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

    // /* 8) Set auto-sleep timeout: e.g., 50 => 0.5s at 100Hz */
    // status = accelerometer_write_reg(MMA8451Q_REG_ASLP_COUNT, 50);
    // if (status != HAL_OK) return status;

    // /* 9) CTRL_REG2: SLPE=1 => auto-sleep, normal mode */
    // uint8_t ctrl_reg2 = CTRL_REG2_SLPE;
    // status = accelerometer_write_reg(MMA8451Q_REG_CTRL_REG2, ctrl_reg2);
    // if (status != HAL_OK) {
    //     return status;
    // }

    // /* 10) CTRL_REG3: IPOL=1 => active-high, WAKE_FF_MT=1 => motion can wake */
    // uint8_t ctrl_reg3 = (CTRL_REG3_IPOL | (1 << 3));
    // status = accelerometer_write_reg(MMA8451Q_REG_CTRL_REG3, ctrl_reg3);
    // if (status != HAL_OK) {
    //     return status;
    // }

    // /* 11) CTRL_REG4: enable interrupts for FF_MT, PULSE, ASLP */
    // uint8_t ctrl_reg4 = (CTRL_REG4_INT_EN_FF_MT |
    //                      CTRL_REG4_INT_EN_PULSE |
    //                      CTRL_REG4_INT_EN_ASLP);
    // status = accelerometer_write_reg(MMA8451Q_REG_CTRL_REG4, ctrl_reg4);
    // if (status != HAL_OK) {
    //     return status;
    // }

    // /* 12) CTRL_REG5: route PULSE => INT1, FF_MT => INT2, ASLP => INT2 */
    // uint8_t ctrl_reg5 = 0x00;
    // // PULSE => bit3 = 1 => INT1
    // ctrl_reg5 |= CTRL_REG5_INT_CFG_PULSE;  // (1 << 3)
    // // FF_MT => bit2 = 0 => INT2
    // // ASLP => bit7 = 0 => INT2
    // status = accelerometer_write_reg(MMA8451Q_REG_CTRL_REG5, ctrl_reg5);
    // if (status != HAL_OK) {
    //     return status;
    // }

    // /* 13) CTRL_REG1: ODR=100Hz (DR=0b011), LNOISE=1, ASLP_RATE=00 => 50Hz, then ACTIVE=1 */
    // uint8_t ctrl_reg1 = 0;
    // // DR=0b011 => bits [5:3] = 3<<3 => 0x18
    // // LNOISE=1 => bit2 => 0x04
    // // ACTIVE => bit0 => 0x01
    // ctrl_reg1 = (3 << 3) | (1 << 2) | CTRL_REG1_ACTIVE;  // 0x1D
    // status = accelerometer_write_reg(MMA8451Q_REG_CTRL_REG1, ctrl_reg1);
    // if (status != HAL_OK) {
    //     return status;
    // }

    // Put device into Active
    status = accel_goto_active();
    if (status != HAL_OK) {
        return status;
    }

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

void accelerometer_handle_int1(void)
{
    led_execute_sequence(LED_SEQ_THREE_BLINKS);
    // PULSE interrupt => reading PULSE_SRC (0x22) clears
    uint8_t dummy;
    accelerometer_read_reg(MMA8451Q_REG_PULSE_SRC, &dummy);
}

void accelerometer_handle_int2(void)
{
    led_execute_sequence(LED_SEQ_FADE_IN_OUT);
    // FF_MT / ASLP => read INT_SOURCE, then read FF_MT_SRC and/or SYSMOD
    uint8_t dummy;
    accelerometer_read_reg(0x0C, &dummy); // INT_SOURCE
    accelerometer_read_reg(0x16, &dummy); // FF_MT_SRC (REG_FF_MT_CFG + 1)
    // Possibly also read SYSMOD (0x0B) if needed for ASLP.
}