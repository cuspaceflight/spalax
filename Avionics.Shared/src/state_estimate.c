#include "state_estimate.h"
#include <calibration.h>
#include <logging.h>
#include <stdbool.h>
#include <math_utils.h>
#include "time_utils.h"
#include <kalman.h>

// Number of data points to calculate the bias and offset of the sensors
#define CALIBRATION_COUNT 100

// The minimum amount of time before another prediction step
#define PREDICTION_THRESHOLD 0.0005f // Half the desired update rate

// Whether to print the acceleration rotated by the computed quaternion
// This is usefull for determining the accuracy of the orientation
#define PRINT_ROTATED_ACCEL


volatile int is_calibrated = false;

float calibration_mag_sum[3] = { 0, 0, 0 };
int calibration_mag_count = 0;
float calibration_accel_sum[3] = { 0, 0, 0 };
int calibration_accel_count = 0;
float accel_bias[3];

float calibration_gyro_sum[3] = { 0, 0, 0 };
int calibration_gyro_count = 0;
float gyro_bias[3]; 

state_estimate_t current_state;
uint64_t last_prediction_time = 0;


void reset_state_estimate(state_estimate_t* estimate) {
    estimate->pos[0] = 0;
    estimate->pos[1] = 0;
    estimate->pos[2] = 0;

    estimate->vel[0] = 0;
    estimate->vel[1] = 0;
    estimate->vel[2] = 0;

    estimate->accel[0] = 0;
    estimate->accel[1] = 0;
    estimate->accel[2] = 0;

    estimate->angular_velocity[0] = 0;
    estimate->angular_velocity[1] = 0;
    estimate->angular_velocity[2] = 0;

    estimate->orientation_q[0] = 0;
    estimate->orientation_q[1] = 0;
    estimate->orientation_q[2] = 0;
    estimate->orientation_q[3] = 1;
}

void print_state_estimate(const state_estimate_t* estimate) {
    PRINT("State estimate { Position: [");
    for (int i = 0; i < 3; ++i)
        PRINT(" %f", estimate->pos[i]);
    PRINT(" ] Velocity: [");
    for (int i = 0; i < 3; ++i)
        PRINT(" %f", estimate->vel[i]);
    PRINT(" ] Acceleration: [");
    for (int i = 0; i < 3; ++i)
        PRINT(" %f", estimate->accel[i]);
    PRINT(" ]}\n");
}

void calibrate(void) {
    float mag_reference[3];

    accel_bias[0] = calibration_accel_sum[0] / (float)calibration_accel_count;
    accel_bias[1] = calibration_accel_sum[1] / (float)calibration_accel_count;
    accel_bias[2] = calibration_accel_sum[2] / (float)calibration_accel_count;

    

    mag_reference[0] = calibration_mag_sum[0] / (float)calibration_mag_count;
    mag_reference[1] = calibration_mag_sum[1] / (float)calibration_mag_count;
    mag_reference[2] = calibration_mag_sum[2] / (float)calibration_mag_count;

    gyro_bias[0] = calibration_gyro_sum[0] / (float)calibration_gyro_count;
    gyro_bias[1] = calibration_gyro_sum[1] / (float)calibration_gyro_count;
    gyro_bias[2] = calibration_gyro_sum[2] / (float)calibration_gyro_count;

    
    PRINT("Accel Reference (%f,%f,%f)\n", accel_bias[0], accel_bias[1], accel_bias[2]);
    PRINT("Mag Reference (%f,%f,%f)\n", mag_reference[0], mag_reference[1], mag_reference[2]);

    //TODO: Set reference vectors
    kalman_init(accel_bias, mag_reference);

    last_prediction_time = get_64bit_time();
    
    is_calibrated = true;
    PRINT("Calibrated!\n");
}

void do_prediction_step(void) {
    uint64_t current_time = get_64bit_time();

    if (current_time <= last_prediction_time)
        return;

    float dt = (float)(current_time - last_prediction_time) / CLOCK_FREQUENCY;

    if (dt < PREDICTION_THRESHOLD)
        return;

    kalman_predict(&current_state, dt);

    last_prediction_time = current_time;
}

void state_estimate_new_accel_raw(const int16_t raw_accel[3]) {
    float accel_calibrated[3];

    calibrate_accel(raw_accel, accel_calibrated);
    //PRINT("Accel (%f, %f, %f)\n", accel_calibrated[0], accel_calibrated[1], accel_calibrated[2]);

    if (is_calibrated) {
        do_prediction_step();
        kalman_new_accel(accel_calibrated);
    } else {
        calibration_accel_sum[0] += accel_calibrated[0];
        calibration_accel_sum[1] += accel_calibrated[1];
        calibration_accel_sum[2] += accel_calibrated[2];
        calibration_accel_count++;

        if (calibration_mag_count > CALIBRATION_COUNT && calibration_accel_count > CALIBRATION_COUNT && calibration_gyro_count > CALIBRATION_COUNT)
            calibrate();
    }
    
}

void state_estimate_new_magnetometer_raw(const int16_t raw_mag[3]) {
    float mag[3];
    //calibrate_mag(raw_mag, mag);

    mag[0] = raw_mag[0];
    mag[1] = raw_mag[1];
    mag[2] = raw_mag[2];

    //PRINT("Mag (%f, %f, %f)\n", mag[0], mag[1], mag[2]);

    if (is_calibrated) {
        do_prediction_step();
        kalman_new_mag(mag);
    } else {
        calibration_mag_sum[0] += mag[0];
        calibration_mag_sum[1] += mag[1];
        calibration_mag_sum[2] += mag[2];
        calibration_mag_count++;

        if (calibration_mag_count > CALIBRATION_COUNT && calibration_accel_count > CALIBRATION_COUNT && calibration_gyro_count > CALIBRATION_COUNT)
            calibrate();
    }
}

void state_estimate_new_pressure_raw(int pressure) {
    if (!is_calibrated)
        return;
    // TODO rotate this using the previous orientation estimate
    // translation_kalman_new_pressure_raw((float)pressure);
}

void state_estimate_new_gyro_raw(const int16_t gyro_raw[3]) {
    float gyro[3];

    calibrate_gyro(gyro_raw, gyro);
    
    
    if (is_calibrated) {
        //PRINT("Gyro: %f %f %f\n", gyro[0], gyro[1], gyro[2]);
        do_prediction_step();
        kalman_new_gyro(gyro);
        
    } else {
        calibration_gyro_sum[0] += gyro[0];
        calibration_gyro_sum[1] += gyro[1];
        calibration_gyro_sum[2] += gyro[2];
        calibration_gyro_count++;

        if (calibration_mag_count > CALIBRATION_COUNT && calibration_accel_count > CALIBRATION_COUNT && calibration_gyro_count > CALIBRATION_COUNT)
            calibrate();
    }
}

void get_state_estimate(state_estimate_t* estimate) {
    *estimate = current_state;
}