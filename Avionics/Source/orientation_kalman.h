#ifndef ORIENTATION_KALMAN
#define ORIENTATION_KALMAN
#include "state_estimate.h"

void orientation_kalman_new_gyro(const float gyro[3]);

// NB quat must be correctly normalised
void orientation_kalman_new_quaternion(const float quat[4]);

void orientation_kalman_prediction_step(state_estimate_t* estimate, float dt);

#endif /* ORIENATION_KALMAN */