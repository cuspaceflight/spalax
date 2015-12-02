#ifndef KALMAN_H
#define KALMAN_H
#include <stdint.h>
#include <state_estimate.h>

void kalman_new_accel(const float accel[3]);

void kalman_new_pressure(float pressure);

void kalman_new_mag(const float mag[3]);

void kalman_new_gyro(const float gyro[3]);

void kalman_predict(state_estimate_t* next_estimate, float dt);

void kalman_init(const float g_ref[3], const float b_ref[3]);

void kalman_set_reference_vectors(const float g_ref[3], const float b_ref[3]);

#endif
