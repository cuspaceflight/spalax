#ifndef KALMAN_H
#define KALMAN_H
#include <stdint.h>
#include <state_estimate.h>

#ifdef __cplusplus
extern "C" {
#endif

void kalman_new_accel(const float accel[3]);

void kalman_new_mag(const float mag[3]);

void kalman_new_gyro(const float gyro[3]);

void kalman_predict(state_estimate_t* next_estimate, float dt);

void kalman_init(const state_estimate_calibration_t* calibration_data);

#ifdef __cplusplus
}
#endif

#endif
