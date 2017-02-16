#ifndef KALMAN_H
#define KALMAN_H
#include <stdint.h>
#include <config/telemetry_packets.h>
#include "compilermacros.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef float fp;

#define KALMAN_NUM_STATES 9
#define KALMAN_ATTITUDE_ERR_IDX 0
#define KALMAN_ANGULAR_VEL_IDX 3
#define KALMAN_GYRO_BIAS_IDX 6

void kalman_init(fp accel_reference[3], fp magno_reference[3], fp initial_orientation[4], fp initial_angular_velocity[3]);

void kalman_predict(fp dt);

void kalman_get_state(state_estimate_t* state);

void kalman_get_covariance(fp covar[12]);

void kalman_new_accel(const fp accel[3]);

void kalman_new_magno(const fp magno[3]);

void kalman_new_gyro(const fp gyro[3]);

#ifdef __cplusplus
}
#endif

#endif
