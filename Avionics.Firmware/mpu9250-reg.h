// mpu9250-reg.h contains human-readable name to register mapping for the
// MPU9250. It also contains useful constants for setting/interpreting those
// register contents.
#pragma once
#ifndef MPU9250_REG_H
#define MPU9250_REG_H

// MPU9250 registers
enum {
    // R/W registers unless noted otherwise

    MPU9250_REG_SELF_TEST_X_GYRO = 0,
    MPU9250_REG_SELF_TEST_Y_GYRO = 1,
    MPU9250_REG_SELF_TEST_Z_GYRO = 2,

    MPU9250_REG_SELF_TEST_X_ACCEL = 13,
    MPU9250_REG_SELF_TEST_Y_ACCEL = 14,
    MPU9250_REG_SELF_TEST_Z_ACCEL = 15,

    MPU9250_REG_XG_OFFSET_H = 19,
    MPU9250_REG_XG_OFFSET_L = 20,
    MPU9250_REG_YG_OFFSET_H = 21,
    MPU9250_REG_YG_OFFSET_L = 22,
    MPU9250_REG_ZG_OFFSET_H = 23,
    MPU9250_REG_ZG_OFFSET_L = 24,

    MPU9250_REG_SMPLRT_DIV = 25,

    MPU9250_REG_CONFIG = 26,
    MPU9250_REG_GYRO_CONFIG = 27,
    MPU9250_REG_ACCEL_CONFIG = 28,
    MPU9250_REG_ACCEL_CONFIG_2 = 29,
    MPU9250_REG_LP_ACCEL_ODR = 30,
    MPU9250_REG_WOM_THR = 31,

    MPU9250_REG_FIFO_EN = 35,

    // TODO: add I2C registers

    MPU9250_REG_INT_PIN_CFG = 55,
    MPU9250_REG_INT_ENABLE = 56,
    MPU9250_REG_INT_STATUS = 58, // read-only

    // Read only output registers

    MPU9250_REG_ACCEL_XOUT_H = 59,
    MPU9250_REG_ACCEL_XOUT_L = 60,
    MPU9250_REG_ACCEL_YOUT_H = 61,
    MPU9250_REG_ACCEL_YOUT_L = 62,
    MPU9250_REG_ACCEL_ZOUT_H = 63,
    MPU9250_REG_ACCEL_ZOUT_L = 64,

    MPU9250_REG_TEMP_OUT_H = 65,
    MPU9250_REG_TEMP_OUT_L = 66,

    MPU9250_REG_GYRO_XOUT_H = 67,
    MPU9250_REG_GYRO_XOUT_L = 68,
    MPU9250_REG_GYRO_YOUT_H = 69,
    MPU9250_REG_GYRO_YOUT_L = 70,
    MPU9250_REG_GYRO_ZOUT_H = 71,
    MPU9250_REG_GYRO_ZOUT_L = 72,

    // TODO: EXT_SENS_... and I2C_SLV... registers

    // R/W again from here unless otherwise noted

    MPU9250_REG_SIGNAL_PATH_RESET = 104,
    MPU9250_REG_MOT_DETECT_CTRL = 105,
    MPU9250_REG_USER_CTRL = 106,
    MPU9250_REG_PWR_MGMT_1 = 107,
    MPU9250_REG_PWR_MGMT_2 = 108,

    MPU9250_REG_FIFO_COUNTH = 114,
    MPU9250_REG_FIFO_COUNTL = 115,
    MPU9250_REG_FIFO_R_W = 116,

    MPU9250_REG_WHO_AM_I = 117, // read-only

    MPU9250_REG_XA_OFFSET_H = 119,
    MPU9250_REG_XA_OFFSET_L = 120,
    MPU9250_REG_YA_OFFSET_H = 122,
    MPU9250_REG_YA_OFFSET_L = 123,
    MPU9250_REG_ZA_OFFSET_H = 125,
    MPU9250_REG_ZA_OFFSET_L = 126,
};

// MPU9250 reset values. All register have a reset value of 0x00 apart from the
// ones noted below.
#define MPU9250_DEFAULT_RESET_VALUE 0x00
#define MPU9250_PWR_MGMT_1_RESET_VALUE 0x01
#define MPU9250_WHO_AM_I_RESET_VALUE 0x71

// bytes_to_uint16 is a macro for taking a high and low byte value and forming a
// uint16_t value from them.
#define bytes_to_uint16(high, low) \
    ( (((uint16_t)(high)) << 8) | ((uint16_t)(low)) )

// bytes_to_int16 is a macro for taking a high and low byte value and forming a
// int16_t value from them.
#define bytes_to_int16(high, low) \
    ( (((int16_t)(high)) << 8) | ((int16_t)(low)) )

// The I2C address of the AK8963 embedded within the MPU9250.
#define MPU9250_AK893_I2C_ADDR 0x0c

#endif // MPU9250_REG_H
