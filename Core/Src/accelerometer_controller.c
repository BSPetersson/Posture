#include "accelerometer_controller.h"
#include "peripherals.h"
#include "led_controller.h"

volatile bool int1_flag = false;
volatile bool int2_flag = false;
volatile accel_data_t latest_accel_data;

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

    value &= ~0x01;  // Clear ACTIVE bit
    return accelerometer_write_reg(MMA8451Q_REG_CTRL_REG1, value);
}

static HAL_StatusTypeDef accel_goto_active(void)
{
    uint8_t value;
    HAL_StatusTypeDef status = accelerometer_read_reg(MMA8451Q_REG_CTRL_REG1, &value);
    if (status != HAL_OK) return status;

    value |= 0x01;  // Set ACTIVE bit
    return accelerometer_write_reg(MMA8451Q_REG_CTRL_REG1, value);
}

// -----------------------------
// Global Variables for Motion State Detection
// -----------------------------
// Independent flags for motion and no-motion conditions.
static volatile bool g_motion_detected = false;
static volatile bool g_no_motion_detected = false;

// Independent timers (0 means not started).
static uint32_t motion_timer_start = 0;
static uint32_t no_motion_timer_start = 0;

// -----------------------------
// Initialization Function
// -----------------------------
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

    // Put device into Active mode
    status = accel_goto_active();
    if (status != HAL_OK) return status;

    // Initialize independent state variables.
    g_motion_detected = false;
    g_no_motion_detected = false;
    motion_timer_start = 0;
    no_motion_timer_start = 0;

    return HAL_OK;
}

// -----------------------------
// Update Function
// -----------------------------
// This function reads the latest acceleration data and independently updates the
// "in motion" and "no motion" flags using separate timers.
// - If the net acceleration deviates from ACCEL_G by more than ACCEL_DEVIATION_THRESHOLD
//   continuously for MOTION_TIME_THRESHOLD_MS, g_motion_detected is set.
// - Separately, if the deviation stays below ACCEL_DEVIATION_THRESHOLD continuously for
//   NO_MOTION_TIME_THRESHOLD_MS, g_no_motion_detected is set.
void accelerometer_controller_update(void)
{
    accel_data_t data;
    HAL_StatusTypeDef status = accelerometer_read_mps2(&data);
    if (status != HAL_OK)
    {
        return;
    }

    latest_accel_data = data;

    // Calculate net acceleration magnitude.
    float mag = sqrtf(data.x_mps2 * data.x_mps2 +
                      data.y_mps2 * data.y_mps2 +
                      data.z_mps2 * data.z_mps2);

    // Compute absolute deviation from gravitational acceleration.
    float deviation = fabsf(mag - ACCEL_G);
    uint32_t now = HAL_GetTick();

    // Check candidate for motion.
    if (deviation > ACCEL_DEVIATION_THRESHOLD)
    {
        // Start (or continue) the motion timer if not already started.
        if (motion_timer_start == 0)
        {
            motion_timer_start = now;
        }
        // If the condition persists, mark motion detected.
        if ((now - motion_timer_start) >= MOTION_TIME_THRESHOLD_MS)
        {
            g_motion_detected = true;
        }
        // Since there is significant movement, reset no-motion timer and flag.
        no_motion_timer_start = 0;
        g_no_motion_detected = false;
    }
    else // Candidate for no motion.
    {
        // Start (or continue) the no-motion timer if not already started.
        if (no_motion_timer_start == 0)
        {
            no_motion_timer_start = now;
        }
        // If the condition persists, mark no motion detected.
        if ((now - no_motion_timer_start) >= NO_MOTION_TIME_THRESHOLD_MS)
        {
            g_no_motion_detected = true;
        }
        // Since the acceleration is steady, reset the motion timer and flag.
        motion_timer_start = 0;
        g_motion_detected = false;
    }
}

// -----------------------------
// Read Acceleration Data
// -----------------------------
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

// -----------------------------
// Clear Interrupts (unchanged)
// -----------------------------
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

// -----------------------------
// Interrupt Handlers
// -----------------------------
void accelerometer_handle_int1(void)
{
    int1_flag = true;
}

void accelerometer_handle_int2(void)
{
    int2_flag = true;
}

// -----------------------------
// Independent Getter Functions for Motion State
// -----------------------------
bool accelerometer_controller_is_in_motion(void)
{
    return g_motion_detected;
}

bool accelerometer_controller_no_motion(void)
{
    return g_no_motion_detected;
}

accel_data_t accelerometer_controller_get_latest_data(void)
{
    return latest_accel_data;
}