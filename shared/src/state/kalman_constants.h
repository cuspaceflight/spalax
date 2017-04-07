#ifndef KALMAN_CONSTANTS_H
#define KALMAN_CONSTANTS_H

#include <stdint.h>
#include <config/telemetry_packets.h>
#include "compilermacros.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef float fp;

#define KALMAN_NUM_STATES 30

#define KALMAN_POSITION_IDX 0
#define KALMAN_VELOCITY_IDX 3
#define KALMAN_ACCELERATION_IDX 6
#define KALMAN_ACCEL_BIAS_IDX 9
#define KALMAN_ATTITUDE_ERR_IDX 12
#define KALMAN_ANGULAR_VEL_IDX 15
#define KALMAN_GYRO_BIAS_IDX 18
#define KALMAN_MAGNO_BIAS_IDX 21
#define KALMAN_MAGNO_REF_IDX 24
#define KALMAN_GYRO_SF_IDX 27

extern fp kalman_accelerometer_cov;
extern fp kalman_magno_cov;
extern fp kalman_gyro_cov;

extern fp initial_position_cov;
extern fp initial_velocity_cov;
extern fp initial_acceleration_cov;
extern fp initial_accel_bias_cov;
extern fp initial_attitude_err_cov;
extern fp initial_angular_vel_cov;
extern fp initial_gyro_bias_cov;
extern fp initial_magno_bias_cov;
extern fp initial_magno_ref_cov;
extern fp initial_gyro_sf_cov;

extern fp position_process_noise;
extern fp velocity_process_noise;
extern fp acceleration_process_noise;
extern fp angular_vel_process_noise;
extern fp gyro_bias_process_noise;
extern fp magno_bias_process_noise;
extern fp accel_bias_process_noise;
extern fp attitude_err_process_noise;
extern fp magno_ref_process_noise;
extern fp gyro_sf_process_noise;

#ifdef __cplusplus
}
#endif

#endif
