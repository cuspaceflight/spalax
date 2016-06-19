#include "state_estimate.h"
#include <calibration.h>
#include <logging.h>
#include <stdbool.h>
#include <math_utils.h>
#include "time_utils.h"
#include <kalman.h>
#include "component_state.h"
#include "messaging.h"
#include "telemetry_config.h"
#include "mpu9250_config.h"

// Number of seconds to compute average sensor bias over
#define CALIBRATION_TIME 5.0f

// The minimum amount of time before another prediction step
#define PREDICTION_THRESHOLD 0.0005f // Half the desired update rate



bool is_calibrated = false;
uint32_t calibration_start_time = 0;
state_estimate_calibration_t calibration_data;

float calibration_mag_sum[3] = { 0, 0, 0 };
float calibration_accel_sum[3] = { 0, 0, 0 };
float calibration_gyro_sum[3] = { 0, 0, 0 };
float calibration_alt_sum = 0;
float altitude_sum = 0;

MESSAGING_PRODUCER(state_estimate_producer_config, telemetry_id_state_estimate_config, sizeof(state_estimate_calibration_t), 2);


state_estimate_t current_state;
uint32_t last_prediction_time = 0;


bool has_mpu9250_config = false;
mpu9250_config_t mpu9250_config;

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

    messaging_producer_send(&state_estimate_producer_config, 0, (const uint8_t*)&calibration_data);

    COMPONENT_STATE_UPDATE(avionics_component_state_estimation, state_ok);
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
        //do_prediction_step();
        //kalman_new_accel(accel);
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
        //do_prediction_step();
        //kalman_new_mag(mag);
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
        //do_prediction_step();
        //kalman_new_gyro(gyro);
    } else {
        calibration_gyro_sum[0] += gyro[0];
        calibration_gyro_sum[1] += gyro[1];
        calibration_gyro_sum[2] += gyro[2];
        calibration_data.gyro_samples++;
    }
}

static bool getPacket(const telemetry_t* packet, message_metadata_t metadata) {
    (void)metadata;

    if (packet->header.id == telemetry_id_mpu9250_data) {
        if (!has_mpu9250_config)
            return true;
        mpu9250_data_t* data = (mpu9250_data_t*)packet->payload;

        mpu9250_calibrated_data_t calibrated;
        mpu9250_calibrate_data(&mpu9250_config, data, &calibrated);
        state_estimate_new_accel(calibrated.accel);
        state_estimate_new_gyro(calibrated.gyro);
        state_estimate_new_magnetometer(calibrated.magno);

        if (!is_calibrated) {
            float dt = (float)(clocks_between(calibration_start_time, platform_get_counter_value())) / CLOCK_FREQUENCY;
            if (dt > CALIBRATION_TIME) {
                calibrate();
            }
        }
    }
    if (packet->header.id == telemetry_id_mpu9250_config) {
        mpu9250_config_t* config = (mpu9250_config_t*)packet->payload;
        mpu9250_config = *config;
        has_mpu9250_config = true;
    }
    return true;
}

MESSAGING_CONSUMER(messaging_consumer, telemetry_source_imu_data, telemetry_source_imu_data_mask, 0, 0, getPacket, 1024);

int32_t state_estimate_thread(void *arg) {
    (void)arg;

    platform_set_thread_name("State Estimation");

    COMPONENT_STATE_UPDATE(avionics_component_state_estimation, state_initializing);

    messaging_producer_init(&state_estimate_producer_config);

    messaging_consumer_init(&messaging_consumer);

    COMPONENT_STATE_UPDATE(avionics_component_state_estimation, state_initializing);

    calibration_start_time = platform_get_counter_value();

    while (true) {
        messaging_consumer_receive(&messaging_consumer, true, false);
    }
}
