#include "accelerometer_controller.h"
#include "haptic_feedback_controller.h"
#include "peripherals.h"
#include "led_controller.h"
#include "posture_math.h"

#define LAST_MEASUREMENTS_HISTORY_SIZE 20
#define SUPER_STILL_ANGLE_THRESHOLD_RADIANS 0.03f
#define SUPER_STILL_MAGNITUDE_THRESHOLD_G 0.1f
#define SUPER_STILL_DURATION_MS 300000
#define ACTIVITY_ANGLE_THRESHOLD_RADIANS 1.0f
#define ACTIVITY_MAGNITUDE_THRESHOLD_G 1.5f
#define ACTIVITY_DURATION_MS 10000
#define ACTIVITY_RESET_DURATION_MS 3000
#define ACTIVITY_TO_NORMAL_DURATION_MS 10000

volatile bool int1_flag = false;
volatile bool int2_flag = false;
volatile accel_data_t latest_accel_data;
volatile float last_measurements_history[LAST_MEASUREMENTS_HISTORY_SIZE][3];
volatile int last_measurements_history_index = 0;
volatile bool last_measurements_history_full = false;
volatile float last_measurements_history_max_angle_from_mean = 0.0f;
volatile float last_measurements_history_max_magnitude_from_mean = 0.0f;
volatile float last_measurements_history_mean_vector[3] = {0.0f, 0.0f, 0.0f};
volatile accelerometer_mode_t accelerometer_mode = NORMAL;

// Add a static variable to track the start time when conditions are first met
static uint32_t super_still_start_time = 0;

// Add static variables to track the start time for activity and reset conditions
static uint32_t activity_start_time = 0;
static uint32_t activity_reset_start_time = 0;

// Add a static variable to track the time when activity conditions fall below the threshold
static uint32_t activity_to_normal_start_time = 0;

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

    return HAL_OK;
}

// -----------------------------
// Update Function
// -----------------------------
void accelerometer_controller_update(void)
{
    uint32_t now = HAL_GetTick();
    
    // Skip accelerometer readings if haptic feedback is active
    if (haptic_feedback_is_active()) {
        return;
    }
    
    accel_data_t accelerometer_data;
    HAL_StatusTypeDef status = accelerometer_read_mps2(&accelerometer_data);
    if (status != HAL_OK)
    {
        return;
    }

    latest_accel_data = accelerometer_data;

    // Store the current vector in the measurements history
    last_measurements_history[last_measurements_history_index][0] = accelerometer_data.x_mps2;
    last_measurements_history[last_measurements_history_index][1] = accelerometer_data.y_mps2;
    last_measurements_history[last_measurements_history_index][2] = accelerometer_data.z_mps2;
    last_measurements_history_index = (last_measurements_history_index + 1) % LAST_MEASUREMENTS_HISTORY_SIZE;
    if (last_measurements_history_index == 0) {
        last_measurements_history_full = true;
    }

    // Calculate the mean of the measurements history
    float magnatude_sum = 0.0f;
    for (int i = 0; i < LAST_MEASUREMENTS_HISTORY_SIZE; i++) {
        last_measurements_history_mean_vector[0] += last_measurements_history[i][0];
        last_measurements_history_mean_vector[1] += last_measurements_history[i][1];
        last_measurements_history_mean_vector[2] += last_measurements_history[i][2];
        magnatude_sum += get_magnitude_of_vector((float *)last_measurements_history[i]);
    }
    last_measurements_history_mean_vector[0] /= LAST_MEASUREMENTS_HISTORY_SIZE;
    last_measurements_history_mean_vector[1] /= LAST_MEASUREMENTS_HISTORY_SIZE;
    last_measurements_history_mean_vector[2] /= LAST_MEASUREMENTS_HISTORY_SIZE;
    float last_measurements_history_mean_magnitude = magnatude_sum / LAST_MEASUREMENTS_HISTORY_SIZE;

    // Calculate the max angle from the mean and the max magnitude from the mean
    if (last_measurements_history_full) {
        float max_angle = 0.0f;
        float max_magnitude_deviation = 0.0f;
        for (int i = 0; i < LAST_MEASUREMENTS_HISTORY_SIZE; i++) {

            // Calculate the angle between the current vector and the mean vector
            float angle = get_angle_between_vectors((float *)last_measurements_history[i], (float *)last_measurements_history_mean_vector);
            if (angle > max_angle) {
                max_angle = angle;
            }

            // Calculate the magnitude of the current vector
            float magnitude = get_magnitude_of_vector((float *)last_measurements_history[i]);
            float deviation = fabs(magnitude - last_measurements_history_mean_magnitude);
            if (deviation > max_magnitude_deviation) {
                max_magnitude_deviation = deviation;
            }
        }
        last_measurements_history_max_magnitude_from_mean = max_magnitude_deviation;
        last_measurements_history_max_angle_from_mean = max_angle;
    }

    // Update the accelerometer mode using a switch statement
    switch (accelerometer_mode) {
        case SUPER_STILL:
            if (int1_flag ||
                last_measurements_history_max_angle_from_mean >= SUPER_STILL_ANGLE_THRESHOLD_RADIANS ||
                last_measurements_history_max_magnitude_from_mean >= SUPER_STILL_MAGNITUDE_THRESHOLD_G) {
                // Transition to NORMAL if int1_flag is true or conditions are no longer met
                accelerometer_mode = NORMAL;
            }
            break;

        case ACTIVITY:
            if (last_measurements_history_max_angle_from_mean <= ACTIVITY_ANGLE_THRESHOLD_RADIANS &&
                last_measurements_history_max_magnitude_from_mean <= ACTIVITY_MAGNITUDE_THRESHOLD_G) {
                if (activity_to_normal_start_time == 0) {
                    // Start the timer to transition back to NORMAL
                    activity_to_normal_start_time = now;
                } else if ((now - activity_to_normal_start_time) >= ACTIVITY_TO_NORMAL_DURATION_MS) {
                    accelerometer_mode = NORMAL;
                    activity_to_normal_start_time = 0;
                }
            } else {
                // Reset the timer if conditions are above threshold
                activity_to_normal_start_time = 0;
            }
            break;

        case NORMAL:
        default:
            // Existing logic for transitioning to SUPER_STILL or ACTIVITY
            if (!int1_flag &&
                last_measurements_history_max_angle_from_mean < SUPER_STILL_ANGLE_THRESHOLD_RADIANS &&
                last_measurements_history_max_magnitude_from_mean < SUPER_STILL_MAGNITUDE_THRESHOLD_G) {
                if (super_still_start_time == 0) {
                    // Start the timer
                    super_still_start_time = now;
                } else if ((now - super_still_start_time) >= SUPER_STILL_DURATION_MS) {
                    // Transition to SUPER_STILL state if the duration is met
                    accelerometer_mode = SUPER_STILL;
                }
            } else {
                // Reset the timer if conditions are not met or int1_flag is true
                super_still_start_time = 0;
            }

            // Check for activity conditions
            if (last_measurements_history_max_angle_from_mean > ACTIVITY_ANGLE_THRESHOLD_RADIANS ||
                last_measurements_history_max_magnitude_from_mean > ACTIVITY_MAGNITUDE_THRESHOLD_G) {
                if (activity_start_time == 0) {
                    // Start the activity timer
                    activity_start_time = now;
                } else if ((now - activity_start_time) >= ACTIVITY_DURATION_MS) {
                    // Transition to ACTIVITY state if the duration is met
                    accelerometer_mode = ACTIVITY;
                }
                // Reset the reset timer
                activity_reset_start_time = 0;
            } else {
                if (activity_reset_start_time == 0) {
                    // Start the reset timer
                    activity_reset_start_time = now;
                } else if ((now - activity_reset_start_time) >= ACTIVITY_RESET_DURATION_MS) { // 3 seconds
                    // Reset the activity timer if conditions are below threshold for 3 seconds
                    activity_start_time = 0;
                }
            }
            break;
    }

    // Reset int1_flag after use
    int1_flag = false;
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
accel_data_t accelerometer_controller_get_latest_data(void)
{
    return latest_accel_data;
}

accelerometer_mode_t accelerometer_controller_get_mode(void)
{
    return accelerometer_mode;
}

bool accelerometer_controller_is_last_measurements_history_full(void)
{
    return last_measurements_history_full;
}

float accelerometer_controller_get_last_measurements_history_max_angle_from_mean(void)
{
    return last_measurements_history_max_angle_from_mean;
}


