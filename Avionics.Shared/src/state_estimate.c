#include "state_estimate.h"
#include <calibration.h>
#include <logging.h>
#include <stdbool.h>
#include <math_utils.h>
#include "time_utils.h"
#include <kalman.h>

// Number of seconds to compute average sensor bias over
#define CALIBRATION_TIME 2.0f

// The minimum amount of time before another prediction step
#define PREDICTION_THRESHOLD 0.0005f // Half the desired update rate

bool is_calibrated = false;
state_estimate_calibration_t calibration_data = {0};

float calibration_mag_sum[3] = { 0, 0, 0 };
float calibration_accel_sum[3] = { 0, 0, 0 };
float calibration_gyro_sum[3] = { 0, 0, 0 };
float calibration_alt_sum = 0;
float altitude_sum = 0;


state_estimate_t current_state;
uint32_t last_prediction_time = 0;

void calibrate(void) {
    calibration_data.accel_bias[0] = calibration_accel_sum[0] / (float)calibration_data.accel_samples;
    calibration_data.accel_bias[1] = calibration_accel_sum[1] / (float)calibration_data.accel_samples;
    calibration_data.accel_bias[2] = calibration_accel_sum[2] / (float)calibration_data.accel_samples;

    calibration_data.mag_bias[0] = calibration_mag_sum[0] / (float)calibration_data.mag_samples;
    calibration_data.mag_bias[1] = calibration_mag_sum[1] / (float)calibration_data.mag_samples;
    calibration_data.mag_bias[2] = calibration_mag_sum[2] / (float)calibration_data.mag_samples;

    calibration_data.gyro_bias[0] = calibration_gyro_sum[0] / (float)calibration_data.gyro_samples;
    calibration_data.gyro_bias[1] = calibration_gyro_sum[1] / (float)calibration_data.gyro_samples;
    calibration_data.gyro_bias[2] = calibration_gyro_sum[2] / (float)calibration_data.gyro_samples;

    kalman_init(&calibration_data);

    is_calibrated = true;
    PRINT("Calibrated!\n");
}

void do_prediction_step(void) {
    uint32_t current_time = platform_get_counter_value();

    float dt = (float)(clocks_between(last_prediction_time, current_time)) / CLOCK_FREQUENCY;

    if (dt < PREDICTION_THRESHOLD)
        return;

    kalman_predict(&current_state, dt);

    last_prediction_time = current_time;
}

// Expects acceleration converted to m/s^2
void state_estimate_new_accel(const float accel[3]) {
    if (is_calibrated) {
        do_prediction_step();
        kalman_new_accel(accel);
    } else {
        calibration_accel_sum[0] += accel[0];
        calibration_accel_sum[1] += accel[1];
        calibration_accel_sum[2] += accel[2];
        calibration_data.accel_samples++;
    }
}

// Doesn't expect any particular unit
void state_estimate_new_magnetometer(const float mag[3]) {
    if (is_calibrated) {
        do_prediction_step();
        kalman_new_mag(mag);
    } else {
        calibration_mag_sum[0] += mag[0];
        calibration_mag_sum[1] += mag[1];
        calibration_mag_sum[2] += mag[2];
        calibration_data.mag_samples++;
    }
}

// Expects altitude in m
void state_estimate_new_altitude(float altitude) {
    if (is_calibrated) {
        // Do something
    } else {
        calibration_alt_sum += altitude;
        calibration_data.alt_samples++;
    }
}

// Expects rotation in rad/s
void state_estimate_new_gyro(const float gyro[3]) {
    if (is_calibrated) {
        do_prediction_step();
        kalman_new_gyro(gyro);
    } else {
        calibration_gyro_sum[0] += gyro[0];
        calibration_gyro_sum[1] += gyro[1];
        calibration_gyro_sum[2] += gyro[2];
        calibration_data.gyro_samples++;
    }
}