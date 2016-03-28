#ifndef MPU9250_CONFIG_H
#define MPU9250_CONFIG_H
#include "compilermacros.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mpu9250_data_t {
    uint16_t accel_x;
    uint16_t accel_y;
    uint16_t accel_z;
    uint16_t temp;
    uint16_t gyro_x;
    uint16_t gyro_y;
    uint16_t gyro_z;
    uint16_t magno_x;
    uint16_t magno_y;
    uint16_t magno_z;
} mpu9250_data_t;

STATIC_ASSERT(sizeof(mpu9250_data_t) == 20, ms5611data_padded);


#ifdef __cplusplus
}
#endif

#endif /* MPU9250_CONFIG_H */
