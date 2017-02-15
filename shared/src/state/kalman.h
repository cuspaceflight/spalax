#ifndef KALMAN_H
#define KALMAN_H
#include <stdint.h>
#include <config/telemetry_packets.h>
#include "compilermacros.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const int kalman_num_states;
extern const int kalman_attitude_err_idx;
extern const int kalman_angular_vel_idx;
extern const int kalman_angular_acc_idx;
extern const int kalman_gyro_bias_idx;

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
