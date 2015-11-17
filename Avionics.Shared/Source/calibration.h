#ifndef CALIBRATION_H
#define CALIBRATION_H
#include <stdint.h>

#define GRAVITY 9.80665f

void calibrate_accel(const int16_t in[3], float out[3]);

void calibrate_mag(const int16_t in[3], float out[3]);

void calibrate_gyro(const int16_t in[3], float out[3]);

float state_estimation_pressure_to_altitude(float pressure);

extern volatile uint8_t calibration_trust_barometer;

#endif /* CALIBRATION_H */