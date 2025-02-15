// TODO:
// use MMA8451Q_REG_STATUS

#include "accelerometer_controller.h"
#include "peripherals.h"
#include "led_controller.h"

volatile bool int1_flag = false;
volatile bool int2_flag = false;

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

    // Configure Transient detection
    status = accelerometer_write_reg(MMA8451Q_REG_TRANSIENT_CFG, TRANSIENT_CFG);

    // Set transient threshold
    status = accelerometer_write_reg(MMA8451Q_REG_TRANSIENT_THS, TRANSIENT_THRESHOLD_VALUE);

    // Set transient debounce count
    status = accelerometer_write_reg(MMA8451Q_REG_TRANSIENT_COUNT, ACCEL_TRANSIENT_DEBOUNCE_COUNT);

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
    HAL_StatusTypeDef status;

    if (!data) {
        return HAL_ERROR;
    }

    uint8_t reg1_value;
    status = accelerometer_read_reg(MMA8451Q_REG_CTRL_REG1, &reg1_value);

    uint8_t xyz_status;
    status = accelerometer_read_reg(MMA8451Q_REG_STATUS, &xyz_status);
    if (status != HAL_OK) {
        return status;
    }

    if (!(xyz_status & 0x08)) {  // Check if ZYXDR bit (bit 3) is set
        return HAL_BUSY;     // No new data available
    }

    // Read MMA8451Q_REG_XYZ_DATA_CFG to determine sensitivity
    uint8_t xyz_data_cfg;
    status = accelerometer_read_reg(MMA8451Q_REG_XYZ_DATA_CFG, &xyz_data_cfg);
    if (status != HAL_OK) {
        return status;
    }

    float sensitivity;
    switch (xyz_data_cfg & 0x03)  // Extract the FS[1:0] bits
    {
        case 0x00: sensitivity = 4096.0f; break;  // ±2g mode
        case 0x01: sensitivity = 2048.0f; break;  // ±4g mode
        case 0x02: sensitivity = 1024.0f; break;  // ±8g mode
        default: return HAL_ERROR;  // Unexpected value
    }

    // Read X, Y, Z (6 bytes)
    uint8_t raw[6];
    status = accelerometer_read_regs(MMA8451Q_REG_OUT_X_MSB, raw, 6);
    if (status != HAL_OK) {
        return status;
    }

    // Convert to 14-bit signed values with explicit sign extension
    int16_t x = (int16_t)((raw[0] << 8) | raw[1]) >> 2;
    int16_t y = (int16_t)((raw[2] << 8) | raw[3]) >> 2;
    int16_t z = (int16_t)((raw[4] << 8) | raw[5]) >> 2;

    // Sign extension for 14-bit values
    if (x & (1 << 13)) x |= 0xC000;  // Extend sign to full 16-bit
    if (y & (1 << 13)) y |= 0xC000;
    if (z & (1 << 13)) z |= 0xC000;

    // Convert to g using detected sensitivity
    float x_g = (float)x / sensitivity;
    float y_g = (float)y / sensitivity;
    float z_g = (float)z / sensitivity;

    // Convert to m/s²
    data->x_mps2 = x_g * ACCEL_G;
    data->y_mps2 = y_g * ACCEL_G;
    data->z_mps2 = z_g * ACCEL_G;

    return HAL_OK;
}

void clear_accelerometer_interrupts(void)
{   
    HAL_StatusTypeDef status;
    uint8_t int_source;

    // Read INT_SOURCE (0x0C) to determine which interrupt(s) triggered
    status = accelerometer_read_reg(MMA8451Q_REG_INT_SOURCE, &int_source);
    if (status != HAL_OK) return;

    // Motion/Freefall interrupt (FF_MT_SRC - 0x16)
    if (int_source & 0x04)  // Motion/Freefall interrupt
    {
        uint8_t ff_mt_src;
        status = accelerometer_read_reg(MMA8451Q_REG_FF_MT_SRC, &ff_mt_src);
        if (status != HAL_OK) return;
    }

    // Transient Motion interrupt (TRANSIENT_SRC - 0x1E)
    if (int_source & 0x20)  // Transient Motion interrupt
    {
        uint8_t transient_src;
        status = accelerometer_read_reg(MMA8451Q_REG_TRANSIENT_SRC, &transient_src);
    }

    // Orientation Change interrupt (PL_STATUS - 0x10)
    if (int_source & 0x10)  // Orientation Change interrupt
    {
        uint8_t pl_status;
        status = accelerometer_read_reg(MMA8451Q_REG_PL_STATUS, &pl_status);
    }

    // Pulse detection interrupt (PULSE_SRC - 0x22)
    if (int_source & 0x08)  // Pulse interrupt
    {
        uint8_t pulse_src;
        status = accelerometer_read_reg(MMA8451Q_REG_PULSE_SRC, &pulse_src);
    }

    // Data Ready interrupt (Read X, Y, Z registers: 0x01 to 0x06)
    if (int_source & 0x01)  // Data Ready interrupt
    {
        uint8_t data[6];
        status = accelerometer_read_regs(MMA8451Q_REG_OUT_X_MSB, data, 6);
    }
}

uint8_t get_sysmod(void)
{
    uint8_t value;
    HAL_StatusTypeDef status = accelerometer_read_reg(MMA8451Q_REG_SYSMOD, &value);
    if (status != HAL_OK) return -1;

    return value;
}

uint8_t get_ff_mt_src(void)
{
    uint8_t value;
    HAL_StatusTypeDef status = accelerometer_read_reg(MMA8451Q_REG_FF_MT_SRC, &value);
    if (status != HAL_OK) return -1;

    return value;
}

uint8_t get_int_source(void)
{
    uint8_t value;
    HAL_StatusTypeDef status = accelerometer_read_reg(MMA8451Q_REG_INT_SOURCE, &value);
    if (status != HAL_OK) return -1;

    return value;
}

uint8_t get_transient_src(void)
{
    uint8_t value;
    HAL_StatusTypeDef status = accelerometer_read_reg(MMA8451Q_REG_TRANSIENT_SRC, &value);
    if (status != HAL_OK) return -1;

    return value;
}

void accelerometer_handle_int1(void)
{
    // // Clear the interrupt
    // uint8_t int_source;
    // accelerometer_read_reg(MMA8451Q_REG_INT_SOURCE, &int_source);

    // // FF_MT_THS
    // uint8_t ff_mt_src;
    // accelerometer_read_reg(MMA8451Q_REG_FF_MT_SRC, &ff_mt_src);

    // Handle the interrupt
    int1_flag = true;
}

void accelerometer_handle_int2(void)
{
    // Clear the interrupt
    // uint8_t int_source;
    // accelerometer_read_reg(MMA8451Q_REG_INT_SOURCE, &int_source);

    // Handle the interrupt
    int2_flag = true;
}