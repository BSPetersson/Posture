#ifndef ACCELEROMETER_CONTROLLER_H
#define ACCELEROMETER_CONTROLLER_H

#include "stm32f0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// -----------------------------
// Register Addresses
// -----------------------------

#define MMA8451Q_REG_STATUS            0x00
#define MMA8451Q_REG_OUT_X_MSB         0x01
#define MMA8451Q_REG_OUT_X_LSB         0x02
#define MMA8451Q_REG_OUT_Y_MSB         0x03
#define MMA8451Q_REG_OUT_Y_LSB         0x04
#define MMA8451Q_REG_OUT_Z_MSB         0x05
#define MMA8451Q_REG_OUT_Z_LSB         0x06
#define MMA8451Q_REG_WHO_AM_I          0x0D
#define MMA8451Q_REG_XYZ_DATA_CFG      0x0E
#define MMA8451Q_REG_HP_FILTER_CUTOFF  0x0F
#define MMA8451Q_REG_FF_MT_CFG         0x15
#define MMA8451Q_REG_FF_MT_THS         0x17
#define MMA8451Q_REG_FF_MT_COUNT       0x18
#define MMA8451Q_REG_PULSE_CFG         0x21
#define MMA8451Q_REG_PULSE_SRC         0x22
#define MMA8451Q_REG_PULSE_THSX        0x23
#define MMA8451Q_REG_PULSE_THSY        0x24
#define MMA8451Q_REG_PULSE_THSZ        0x25
#define MMA8451Q_REG_PULSE_TMLT        0x26
#define MMA8451Q_REG_PULSE_LTCY        0x27
#define MMA8451Q_REG_PULSE_WIND        0x28
#define MMA8451Q_REG_ASLP_COUNT        0x29
#define MMA8451Q_REG_CTRL_REG1         0x2A
#define MMA8451Q_REG_CTRL_REG2         0x2B
#define MMA8451Q_REG_CTRL_REG3         0x2C
#define MMA8451Q_REG_CTRL_REG4         0x2D
#define MMA8451Q_REG_CTRL_REG5         0x2E

// -----------------------------
// Mode Values
// -----------------------------

#define XYZ_DATA_CFG_HP_OFF_FS_2G  0x00 // No high-pass filter, ±2g
#define XYZ_DATA_CFG_HP_ON_FS_2G   0x10 // with high-pass filter, ±2g

#define HP_FILTER_CUTOFF           0x00

// -----------------------------
// Register Values
// -----------------------------

#define WHO_AM_I_VALUE        0x1A

// -----------------------------
// I2C Address
// -----------------------------

#ifndef MMA8451Q_I2C_ADDR
#define MMA8451Q_I2C_ADDR   (0x1C << 1)
#endif

// -----------------------------
// Constants
// -----------------------------

// 14-bit ±2g => 4096 counts/g
#ifndef MMA8451Q_SENS_2G_14BIT
#define MMA8451Q_SENS_2G_14BIT  4096.0f
#endif

// Acceleration due to gravity (m/s^2)
#ifndef ACCEL_G
#define ACCEL_G 9.80665f
#endif

#define FF_MT_CFG_ELE       (1 << 7)
#define FF_MT_CFG_OAE       (1 << 6)
#define FF_MT_CFG_XEFE      (1 << 3)
#define FF_MT_CFG_YEFE      (1 << 2)
#define FF_MT_CFG_ZEFE      (1 << 1)


#define PULSE_CFG_ELE       (1 << 6)
#define PULSE_CFG_ZSPEFE    (1 << 3)
#define PULSE_CFG_YSPEFE    (1 << 1)
#define PULSE_CFG_XSPEFE    (1 << 0)



#define CTRL_REG1_ACTIVE    (1 << 0)
#define CTRL_REG1_F_READ    (1 << 1)
#define CTRL_REG1_LNOISE    (1 << 2)

#define CTRL_REG2_SLPE      (1 << 2)

#define CTRL_REG3_IPOL      (1 << 1)

#define CTRL_REG4_INT_EN_FF_MT   (1 << 2)
#define CTRL_REG4_INT_EN_PULSE   (1 << 3)
#define CTRL_REG4_INT_EN_ASLP    (1 << 7)

#define CTRL_REG5_INT_CFG_FF_MT  (1 << 2)
#define CTRL_REG5_INT_CFG_PULSE  (1 << 3)
#define CTRL_REG5_INT_CFG_ASLP   (1 << 7)

/* MMA8451Q SA0=0 => 7-bit address=0x1C => 8-bit = 0x38
   If SA0=1, use (0x1D << 1). */
#ifndef MMA8451Q_I2C_ADDR
#define MMA8451Q_I2C_ADDR   (0x1C << 1)
#endif

/* Default thresholds / config: you can tweak as needed */
#ifndef ACCEL_MOTION_THRESHOLD_MG
#define ACCEL_MOTION_THRESHOLD_MG  200
#endif

#ifndef ACCEL_TAP_THRESHOLD_MG
#define ACCEL_TAP_THRESHOLD_MG     600
#endif

#ifndef ACCEL_MOTION_DEBOUNCE_COUNT
#define ACCEL_MOTION_DEBOUNCE_COUNT  5
#endif

#ifndef ACCEL_TAP_TIME_LIMIT
#define ACCEL_TAP_TIME_LIMIT        10
#endif

#ifndef ACCEL_TAP_LATENCY
#define ACCEL_TAP_LATENCY           20
#endif

#ifndef ACCEL_TAP_WINDOW
#define ACCEL_TAP_WINDOW            80
#endif

// Accelerometer Data Structure
typedef struct {
    float x_mps2;
    float y_mps2;
    float z_mps2;
} accel_data_t;

// Initialization
HAL_StatusTypeDef accelerometer_controller_initialize(void);

// Read Accelerometer Data
HAL_StatusTypeDef accelerometer_read_mps2(accel_data_t *data);

// Interrupt Handlers
void accelerometer_handle_int1(void);
void accelerometer_handle_int2(void);

#endif // ACCELEROMETER_CONTROLLER_H