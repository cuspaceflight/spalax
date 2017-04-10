#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>
#include <config/telemetry_packets.h>
#include "compilermacros.h"
#include "kalman_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

void kalman_init(const fp accel_reference[3], const fp magno_reference[3], const fp initial_orientation[4],
                 const fp initial_angular_velocity[3], const fp initial_position[3], const fp initial_velocity[3],
                 const fp initial_acceleration[3], const fp initial_gyro_bias[3], const fp initial_accel_bias[3],
                 const fp initial_magno_bias[3]);

void kalman_predict(fp dt);

void kalman_get_state(state_estimate_t *state);

void kalman_get_state_debug(state_estimate_debug_t* state);

void kalman_get_covariance(fp covar[12]);

void kalman_new_accel(const fp accel[3]);

void kalman_new_magno(const fp magno[3]);

void kalman_new_gyro(const fp gyro[3]);

#ifdef __cplusplus
}
#endif

#endif
