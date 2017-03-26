#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>
#include <config/telemetry_packets.h>
#include "compilermacros.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef float fp;

#define KALMAN_NUM_STATES 18
#define KALMAN_ATTITUDE_ERR_IDX 0
#define KALMAN_ANGULAR_VEL_IDX 3
#define KALMAN_GYRO_BIAS_IDX 6
#define KALMAN_POSITION_IDX 9
#define KALMAN_VELOCITY_IDX 12
#define KALMAN_ACCELERATION_IDX 15

extern fp kalman_accelerometer_cov;
extern fp kalman_magno_cov;
extern fp kalman_gyro_cov;

void
kalman_init(const fp accel_reference[3], const fp magno_reference[3], const fp initial_orientation[4], const fp initial_angular_velocity[3],
            const fp initial_position[3], const fp initial_velocity[3], const fp initial_acceleration[3]);

void kalman_predict(fp dt);

void kalman_get_state(state_estimate_t *state);

void kalman_get_gyro_bias(fp bias[3]);

void kalman_get_covariance(fp covar[12]);

void kalman_new_accel(const fp accel[3]);

void kalman_new_magno(const fp magno[3]);

void kalman_new_gyro(const fp gyro[3]);

#ifdef __cplusplus
}
#endif

#endif
