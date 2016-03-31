#ifndef MPU9250_CONFIG_H
#define MPU9250_CONFIG_H
#include "compilermacros.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mpu9250_data_t {
    int16_t accel[3];
    int16_t temp;
    int16_t gyro[3];
} mpu9250_data_t;

STATIC_ASSERT(sizeof(mpu9250_data_t) == 14, ms5611data_padded);

static const int mpu9250_send_over_usb_count = 100; // Will send 1 in every 100

#ifdef __cplusplus
}
#endif

#endif /* MPU9250_CONFIG_H */
