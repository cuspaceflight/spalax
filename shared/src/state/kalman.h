#ifndef KALMAN_H
#define KALMAN_H
#include <stdint.h>
#include <config/telemetry_packets.h>
#include "compilermacros.h"

#ifdef __cplusplus
extern "C" {
#endif

#define KALMAN_NUM_STATES 9
#define KALMAN_ATTITUDE_ERR_IDX 0
#define KALMAN_ANGULAR_VEL_IDX 3
#define KALMAN_GYRO_BIAS_IDX 6

void kalman_init(float accel_reference[3], float magno_reference[3]);

void kalman_predict(float dt);

void kalman_get_state(state_estimate_t* state);

void kalman_get_covariance(float covar[12]);

void kalman_new_accel(const float accel[3]);

void kalman_new_magno(const float magno[3]);

void kalman_new_gyro(const float gyro[3]);

#ifdef __cplusplus
}
#endif

#endif
