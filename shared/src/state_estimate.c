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
#include <avionics_config.h>
#include <board_config.h>

// Number of seconds to compute average sensor bias over
#define CALIBRATION_TIME 5.0f

#define STATUS_TIME 1.0f

// The minimum amount of time before another prediction step
#define PREDICTION_THRESHOLD 0.0005f // Half the desired update rate

static bool is_calibrated = false;
static uint32_t calibration_start_time = 0;
static state_estimate_calibration_t calibration_data;

static float calibration_mag_sum[3] = {0, 0, 0};
static uint16_t calibration_mag_samples = 0;

static float calibration_accel_sum[3] = {0, 0, 0};
static uint16_t calibration_accel_samples = 0;

static float calibration_gyro_sum[3] = {0, 0, 0};
static uint16_t calibration_gyro_samples = 0;

static float calibration_alt_sum = 0;
static uint16_t calibration_alt_samples = 0;

MESSAGING_PRODUCER(state_estimate_producer_config, telemetry_id_state_estimate_config, sizeof(state_estimate_calibration_t), 2);
MESSAGING_PRODUCER(state_estimate_producer_data, telemetry_id_state_estimate_data, sizeof(state_estimate_t), 40);
MESSAGING_PRODUCER(state_estimate_producer_status, telemetry_id_state_estimate_status, sizeof(state_estimate_status_t), 2);

static state_estimate_t current_state;
static state_estimate_status_t current_status;
static uint32_t last_prediction_time = 0;
static uint32_t last_status_time = 0;

#define state_estimate_send_over_can_count 0

bool has_mpu9250_config = false;
mpu9250_config_t mpu9250_config;

//static void send_calibration_data(void) {
//    messaging_producer_send(&state_estimate_producer_config, 0, (const uint8_t*)&calibration_data);
//}
//
//static void calibrate(float sample_time) {
//    if (is_calibrated) {
//        COMPONENT_STATE_UPDATE(avionics_component_state_estimation, state_error);
//        return;
//    }
//
//	calibration_data.accel_bias[0] = calibration_accel_sum[0] / (float)calibration_accel_samples;
//	calibration_data.accel_bias[1] = calibration_accel_sum[1] / (float)calibration_accel_samples;
//	calibration_data.accel_bias[2] = calibration_accel_sum[2] / (float)calibration_accel_samples;
//
//	calibration_data.mag_bias[0] = calibration_mag_sum[0] / (float)calibration_mag_samples;
//	calibration_data.mag_bias[1] = calibration_mag_sum[1] / (float)calibration_mag_samples;
//	calibration_data.mag_bias[2] = calibration_mag_sum[2] / (float)calibration_mag_samples;
//
//	calibration_data.sample_time = sample_time;
//
//	//calibration_data.gyro_bias[0] = calibration_gyro_sum[0] / (float)calibration_gyro_samples;
//	//calibration_data.gyro_bias[1] = calibration_gyro_sum[1] / (float)calibration_gyro_samples;
//	//calibration_data.gyro_bias[2] = calibration_gyro_sum[2] / (float)calibration_gyro_samples;
//
//	//calibration_data.alt_offset = calibration_alt_sum / (float)calibration_alt_samples;
//
//
//	kalman_init(&calibration_data);
//
//	last_status_time = platform_get_counter_value();
//	current_status.sample_time = 0;
//	current_status.number_prediction_steps = 0;
//	current_status.number_update_steps = 0;
//
//	is_calibrated = true;
//
//	send_calibration_data();
//
//	COMPONENT_STATE_UPDATE(avionics_component_state_estimation, state_ok);
//}

static void send_state_estimate(void) {
	static uint32_t usb_send_count = 0;
	static uint32_t can_send_count = state_estimate_send_over_can_count;
	message_metadata_t flags = 0;

	if (usb_send_count == 100) {
		usb_send_count = 0;
	} else {
		flags |= message_flags_dont_send_over_usb;
		usb_send_count++;
	}

	if (can_send_count == state_estimate_send_over_can_count) {
		can_send_count = 0;
		flags |= message_flags_send_over_can;
	} else {
		can_send_count++;
	}


	messaging_producer_send(&state_estimate_producer_data, flags, (const uint8_t*)&current_state);
}

static void do_prediction_step(void) {
	uint32_t current_time = platform_get_counter_value();

	float dt = (float)(clocks_between(last_prediction_time, current_time)) / CLOCK_FREQUENCY;

	if (dt < PREDICTION_THRESHOLD)
		return;

	kalman_predict(&current_state, dt);

	current_status.number_prediction_steps++;

	last_prediction_time = current_time;
}

static void send_current_status(void) {
	uint32_t current_time = platform_get_counter_value();

	float dt = (float)(clocks_between(last_status_time, current_time)) / CLOCK_FREQUENCY;

	if (dt < STATUS_TIME)
		return;

	current_status.sample_time = dt;
	messaging_producer_send(&state_estimate_producer_status, 0, (const uint8_t*)&current_status);
	current_status.sample_time = 0;
	current_status.number_prediction_steps = 0;
	current_status.number_update_steps = 0;
	last_status_time = platform_get_counter_value();
}

static void state_estimate_new_imu_measurement(const float accel[3], const float mag[3], const float gyro[3]) {
	if (is_calibrated) {
		do_prediction_step();

		kalman_new_accel(accel);
		kalman_new_mag(mag);
		kalman_new_gyro(gyro);

		current_status.number_update_steps++;


		send_state_estimate();
		send_current_status();
	} else {
		calibration_accel_sum[0] += accel[0];
		calibration_accel_sum[1] += accel[1];
		calibration_accel_sum[2] += accel[2];
		calibration_accel_samples++;

		calibration_mag_sum[0] += mag[0];
		calibration_mag_sum[1] += mag[1];
		calibration_mag_sum[2] += mag[2];
		calibration_mag_samples++;

		calibration_gyro_sum[0] += gyro[0];
		calibration_gyro_sum[1] += gyro[1];
		calibration_gyro_sum[2] += gyro[2];
		calibration_gyro_samples++;
	}
}

// Expects altitude in m
static void state_estimate_new_altitude(float altitude) {
	if (is_calibrated) {
		// Do something
	} else {
		calibration_alt_sum += altitude;
		calibration_alt_samples++;
	}
}

static bool getPacket(const telemetry_t* packet, message_metadata_t metadata) {
	(void)metadata;
	// TODO: Devise a better way of doing this
	if (packet->header.origin != local_config.origin)
		return true; // We ignore any packets from other sources

	if (packet->header.id == telemetry_id_mpu9250_data) {
		if (!has_mpu9250_config)
			return true;
		mpu9250_data_t* data = (mpu9250_data_t*)packet->payload;

		mpu9250_calibrated_data_t calibrated;
		mpu9250_calibrate_data(&mpu9250_config, data, &calibrated);

		state_estimate_new_imu_measurement(calibrated.accel, calibrated.magno, calibrated.gyro);

		//        if (!is_calibrated) {
		//            float dt = (float)(clocks_between(calibration_start_time, platform_get_counter_value())) / CLOCK_FREQUENCY;
		//            if (dt > CALIBRATION_TIME) {
		//                calibrate(dt);
		//            }
		//        }
	} else if (packet->header.id == telemetry_id_mpu9250_config) {
		if (has_mpu9250_config) {
			return true;
		}
		mpu9250_config_t* config = (mpu9250_config_t*)packet->payload;
		mpu9250_config = *config;
		has_mpu9250_config = true;
	}
	return true;
}

MESSAGING_CONSUMER(messaging_consumer, telemetry_source_imu_data, telemetry_source_imu_data_mask, 0, 0, getPacket, 1024);

void calibrate_from_references(void) {
	if (is_calibrated) {
		COMPONENT_STATE_UPDATE(avionics_component_state_estimation, state_error);
		return;
	}

	board_config_t* config = getBoardConfig();

	for (int i = 0; i < 3; i++) {
		calibration_data.accel_bias[i] = config->accel_reference[i];
		calibration_data.mag_bias[i] = config->magno_reference[i];
	}

	calibration_data.sample_time = 0;

	//calibration_data.gyro_bias[0] = calibration_gyro_sum[0] / (float)calibration_gyro_samples;
	//calibration_data.gyro_bias[1] = calibration_gyro_sum[1] / (float)calibration_gyro_samples;
	//calibration_data.gyro_bias[2] = calibration_gyro_sum[2] / (float)calibration_gyro_samples;

	//calibration_data.alt_offset = calibration_alt_sum / (float)calibration_alt_samples;


	kalman_init(&calibration_data);

	last_status_time = platform_get_counter_value();
	current_status.sample_time = 0;
	current_status.number_prediction_steps = 0;
	current_status.number_update_steps = 0;

	is_calibrated = true;

	COMPONENT_STATE_UPDATE(avionics_component_state_estimation, state_ok);
}

void state_estimate_thread(void* arg) {
	(void)arg;

	platform_set_thread_name("State Estimation");

	COMPONENT_STATE_UPDATE(avionics_component_state_estimation, state_initializing);

	messaging_producer_init(&state_estimate_producer_config);
	messaging_producer_init(&state_estimate_producer_data);
	messaging_producer_init(&state_estimate_producer_status);

	messaging_consumer_init(&messaging_consumer);

	COMPONENT_STATE_UPDATE(avionics_component_state_estimation, state_initializing);

	calibration_start_time = platform_get_counter_value();

	calibrate_from_references();

	while (true) {
		messaging_consumer_receive(&messaging_consumer, true, false);
	}
}
