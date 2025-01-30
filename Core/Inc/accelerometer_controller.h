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
#define MMA8451Q_REG_SYSMOD            0x0B
#define MMA8451Q_REG_INT_SOURCE        0x0C
#define MMA8451Q_REG_WHO_AM_I          0x0D
#define MMA8451Q_REG_XYZ_DATA_CFG      0x0E
#define MMA8451Q_REG_HP_FILTER_CUTOFF  0x0F
#define MMA8451Q_REG_FF_MT_CFG         0x15
#define MMA8451Q_REG_FF_MT_SRC         0x16
#define MMA8451Q_REG_FF_MT_THS         0x17
#define MMA8451Q_REG_FF_MT_COUNT       0x18
#define MMA8451Q_REG_TRANSIENT_SRC     0x1E
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

// Setting CTRL_REG1 register values
#define CTRL_REG1_ASLP_RATE1 (0 << 7)
#define CTRL_REG1_ASLP_RATE0 (0 << 6)
#define CTRL_REG1_DR2        (0 << 5)
#define CTRL_REG1_DR1        (1 << 4)
#define CTRL_REG1_DR0        (1 << 3)
#define CTRL_REG1_LNOISE     (0 << 2)
#define CTRL_REG1_F_READ     (0 << 1)
#define CTRL_REG1_ACTIVE     (0 << 0)
#define CTRL_REG1 (CTRL_REG1_ASLP_RATE1 | CTRL_REG1_ASLP_RATE0 | CTRL_REG1_DR2 | CTRL_REG1_DR1 | CTRL_REG1_DR0 | CTRL_REG1_LNOISE | CTRL_REG1_F_READ | CTRL_REG1_ACTIVE)

// Setting CTRL_REG2 register values
#define CTRL_REG2_ST     (0 << 7)
#define CTRL_REG2_RST    (0 << 6)
#define CTRL_REG2_SMODS1 (1 << 4)
#define CTRL_REG2_SMODS0 (1 << 3)
#define CTRL_REG2_SLPE   (1 << 2)
#define CTRL_REG2_MODS1  (0 << 1)
#define CTRL_REG2_MODS0  (0 << 0)
#define CTRL_REG2 (CTRL_REG2_ST | CTRL_REG2_RST | CTRL_REG2_SMODS1 | CTRL_REG2_SMODS0 | CTRL_REG2_SLPE | CTRL_REG2_MODS1 | CTRL_REG2_MODS0)

// Setting CTRL_REG3 register values
#define CTRL_REG3_FIFO_GATE (0 << 7)
#define CTRL_REG3_WAKE_TRANS (1 << 6)
#define CTRL_REG3_WAKE_LNDPRT (0 << 5)
#define CTRL_REG3_WAKE_PULSE (1 << 4)
#define CTRL_REG3_WAKE_FF_MT (1 << 3)
#define CTRL_REG3_IPOL (1 << 1)
#define CTRL_REG3_PP_OD (0 << 0)
#define CTRL_REG3 (CTRL_REG3_FIFO_GATE | CTRL_REG3_WAKE_TRANS | CTRL_REG3_WAKE_LNDPRT | CTRL_REG3_WAKE_PULSE | CTRL_REG3_WAKE_FF_MT | CTRL_REG3_IPOL | CTRL_REG3_PP_OD)

// Setting CTRL_REG4 register values
#define CTRL_REG4_INT_EN_ASLP    (1 << 7)
#define CTRL_REG4_INT_EN_FIFO    (0 << 6)
#define CTRL_REG4_INT_EN_TRANS   (0 << 5)
#define CTRL_REG4_INT_EN_LNDPRT  (0 << 4)
#define CTRL_REG4_INT_EN_PULSE   (0 << 3)
#define CTRL_REG4_INT_EN_FF_MT   (1 << 2)
#define CTRL_REG4_INT_EN_DRDY    (0 << 0)
#define CTRL_REG4 (CTRL_REG4_INT_EN_ASLP | CTRL_REG4_INT_EN_FIFO | CTRL_REG4_INT_EN_TRANS | CTRL_REG4_INT_EN_LNDPRT | CTRL_REG4_INT_EN_PULSE | CTRL_REG4_INT_EN_FF_MT | CTRL_REG4_INT_EN_DRDY)

// Setting CTRL_REG5 register values
#define CTRL_REG5_INT_CFG_ASLP   (1 << 7)
#define CTRL_REG5_INT_CFG_FIFO   (0 << 6)
#define CTRL_REG5_INT_CFG_TRANS  (0 << 5)
#define CTRL_REG5_INT_CFG_LNDPRT (0 << 4)
#define CTRL_REG5_INT_CFG_PULSE  (0 << 3)
#define CTRL_REG5_INT_CFG_FF_MT  (1 << 2)
#define CTRL_REG5_INT_CFG_DRDY   (0 << 0)
#define CTRL_REG5 (CTRL_REG5_INT_CFG_ASLP | CTRL_REG5_INT_CFG_FIFO | CTRL_REG5_INT_CFG_TRANS | CTRL_REG5_INT_CFG_LNDPRT | CTRL_REG5_INT_CFG_PULSE | CTRL_REG5_INT_CFG_FF_MT | CTRL_REG5_INT_CFG_DRDY)

// Setting XYZ_DATA_CFG register values
#define XYZ_DATA_CFG_HPF_OUT    (0 << 4)
#define XYZ_DATA_CFG_FS1        (0 << 1)
#define XYZ_DATA_CFG_FS0        (0 << 0)
#define XYZ_DATA_CFG (XYZ_DATA_CFG_HPF_OUT | XYZ_DATA_CFG_FS1 | XYZ_DATA_CFG_FS0)

// Setting HP_FILTER_CUTOFF register values
#define HP_FILTER_CUTOFF_Pulse_HPF_BYP (0 << 5)
#define HP_FILTER_CUTOFF_Pulse_LPF_EN  (0 << 4)
#define HP_FILTER_CUTOFF_SEL1          (0 << 1)
#define HP_FILTER_CUTOFF_SEL0          (0 << 0)
#define HP_FILTER_CUTOFF (HP_FILTER_CUTOFF_Pulse_HPF_BYP | HP_FILTER_CUTOFF_Pulse_LPF_EN | HP_FILTER_CUTOFF_SEL1 | HP_FILTER_CUTOFF_SEL0)

// Setting FF_MT_CFG register values
#define FF_MT_CFG_ELE       (1 << 7)
#define FF_MT_CFG_OAE       (1 << 6)
#define FF_MT_CFG_ZEFE      (1 << 5)
#define FF_MT_CFG_YEFE      (1 << 4)
#define FF_MT_CFG_XEFE      (1 << 3)
#define FF_MT_CFG (FF_MT_CFG_ELE | FF_MT_CFG_OAE | FF_MT_CFG_ZEFE | FF_MT_CFG_YEFE | FF_MT_CFG_XEFE)

// Accelerometer Motion Detection Threshold (mg). ACCEL_MOTION_THRESHOLD_MG * 0.063 mg/LSB = threshold in mg
#define ACCEL_MOTION_THRESHOLD 6
#define ACCEL_MOTION_DBCNTM (0 << 7)
#define ACCEL_MOTION_THRESHOLD_VALUE (ACCEL_MOTION_DBCNTM | (ACCEL_MOTION_THRESHOLD << 1))

// Accelerometer Motion Debounce Count
#ifndef ACCEL_MOTION_DEBOUNCE_COUNT
#define ACCEL_MOTION_DEBOUNCE_COUNT  3
#endif

// Auto-Sleep Timeout
#ifndef ASLP_COUNT
#define ASLP_COUNT 50
#endif

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

#define PULSE_CFG_ELE       (1 << 6)
#define PULSE_CFG_ZSPEFE    (1 << 3)
#define PULSE_CFG_YSPEFE    (1 << 1)
#define PULSE_CFG_XSPEFE    (1 << 0)








#ifndef ACCEL_TAP_THRESHOLD_MG
#define ACCEL_TAP_THRESHOLD_MG     600
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

HAL_StatusTypeDef accelerometer_controller_initialize(void);
HAL_StatusTypeDef accelerometer_read_mps2(accel_data_t *data);

uint8_t get_sysmod(void);
uint8_t get_ff_mt_src(void);
uint8_t get_int_source(void);
uint8_t get_transient_src(void);

// void accelerometer_handle_int1(void);
// void accelerometer_handle_int2(void);

#endif // ACCELEROMETER_CONTROLLER_H