/*
* Translational State estimation and sensor fusion
* Avionics14
* 2014 Raphael Taylor-Davies, Cambridge University Spaceflight
*/

#ifndef TRANSLATION_KALMAN_H
#define TRANSLATION_KALMAN_H

#include <stdint.h>
#include <state_estimate.h>

void translation_kalman_prediction_step(state_estimate_t* state, float dt);

void translation_kalman_new_pressure_raw(float pressure);

// The scaled and calibrated acceleration vector in global coordinates with gravity removed
void translation_kalman_new_accel(const float* accel);



#endif /* TRANSLATION_KALMAN_H */