#include "state_estimate.h"
#include <calibration.h>
#include <logging.h>
#include <stdbool.h>
#include <math_utils.h>

// Define these to add locking primitives to protect the data shared between threads
#define LOCK_STATE_ESTIMATE
#define UNLOCK_STATE_ESTIMATE

// Number of data points to calculate the bias and offset of the sensors
#define CALIBRATION_COUNT 100

// Whether to use the QUEST algorithm or the mag rotation estimator
//#define USE_QUEST

// Whether to run the orientation kalman - disabling will use just the quaternion estimator
#define USE_ORIENTATION_KALMAN

// Whether to print the acceleration rotated by the computed quaternion
// This is usefull for determining the accuracy of the orientation
//#define PRINT_ROTATED_ACCEL


volatile int is_calibrated = false;

float calibration_mag_sum[3] = { 0, 0, 0 };
int calibration_mag_count = 0;
float calibration_accel_sum[3] = { 0, 0, 0 };
int calibration_accel_count = 0;
float accel_bias[3];

float calibration_gyro_sum[3] = { 0, 0, 0 };
int calibration_gyro_count = 0;
float gyro_bias[3];

float accel_calibrated[3];

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

void calibrate() {
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

    //TODO: Set reference vectors
//#ifdef USE_QUEST
//	quest_estimator_set_reference_vectors(accel_bias, mag_reference);
//#else
//	mag_rotation_estimator_set_reference_vector(mag_reference);
//#endif
    
	is_calibrated = true;
	PRINT("Calibrated!\n");
}

void state_estimate_new_accel_raw(const int16_t raw_accel[3]) {
	LOCK_STATE_ESTIMATE;
	calibrate_accel(raw_accel, accel_calibrated);

	accel_calibrated[0] = accel_calibrated[0];
	accel_calibrated[1] = accel_calibrated[1];
	accel_calibrated[2] = accel_calibrated[2];

	if (is_calibrated) {
		

		
//#ifdef USE_QUEST
//		quest_estimator_new_accel(accel_calibrated);
//#endif
		//PRINT("Accel %f %f %f\n", accel[0], accel[1], accel[2]);
		//accel_calibrated[0] -= accel_bias[0];
		//accel_calibrated[1] -= accel_bias[1];
		//accel_calibrated[2] -= accel_bias[2];
		//accel[1] -= GRAVITY;
		//PRINT("Accel %f %f %f\n", accel[0], accel[1], accel[2]);



		//translation_kalman_new_accel(accel_calibrated);
	} else {
		calibration_accel_sum[0] += accel_calibrated[0];
		calibration_accel_sum[1] += accel_calibrated[1];
		calibration_accel_sum[2] += accel_calibrated[2];
		calibration_accel_count++;

		if (calibration_mag_count > CALIBRATION_COUNT && calibration_accel_count > CALIBRATION_COUNT && calibration_gyro_count > CALIBRATION_COUNT)
			calibrate();
	}
	
	
	
	UNLOCK_STATE_ESTIMATE;
}

void state_estimate_new_magnetometer_raw(const int16_t raw_mag[3]) {
	LOCK_STATE_ESTIMATE;

	float mag[3];
	//calibrate_mag(raw_mag, mag);

	mag[0] = raw_mag[0];
	mag[1] = raw_mag[1];
	mag[2] = raw_mag[2];


	if (is_calibrated) {
		//PRINT("Mag %f %f %f\n", mag[0], mag[1], mag[2]);

        //float magnitude = sqrtf(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
        //PRINT("%.2f\n", magnitude);

//#ifdef USE_QUEST
//		quest_estimator_new_mag(mag);
//#else
//		mag_rotation_estimator_new_mag(mag);
//#endif
	} else {
		calibration_mag_sum[0] += mag[0];
		calibration_mag_sum[1] += mag[1];
		calibration_mag_sum[2] += mag[2];
		calibration_mag_count++;

		if (calibration_mag_count > CALIBRATION_COUNT && calibration_accel_count > CALIBRATION_COUNT && calibration_gyro_count > CALIBRATION_COUNT)
			calibrate();
	}

	UNLOCK_STATE_ESTIMATE;
}

void state_estimate_new_pressure_raw(int pressure) {
	if (!is_calibrated)
		return;
	LOCK_STATE_ESTIMATE;
    // TODO rotate this using the previous orientation estimate
	// translation_kalman_new_pressure_raw((float)pressure);
	UNLOCK_STATE_ESTIMATE;
}

void state_estimate_new_gyro_raw(const int16_t gyro_raw[3]) {
	LOCK_STATE_ESTIMATE;
	float gyro[3];
	gyro[0] -= gyro_bias[0];
	gyro[1] -= gyro_bias[1];
	gyro[2] -= gyro_bias[2];

	calibrate_gyro(gyro_raw, gyro);
	if (is_calibrated) {
		//PRINT("Gyro: %f %f %f\n", gyro[0], gyro[1], gyro[2]);

		//orientation_kalman_new_gyro(gyro);
	} else {
		calibration_gyro_sum[0] += gyro[0];
		calibration_gyro_sum[1] += gyro[1];
		calibration_gyro_sum[2] += gyro[2];
		calibration_gyro_count++;

		if (calibration_mag_count > CALIBRATION_COUNT && calibration_accel_count > CALIBRATION_COUNT && calibration_gyro_count > CALIBRATION_COUNT)
			calibrate();
	}
	UNLOCK_STATE_ESTIMATE;
}

void state_estimate_compute_next(state_estimate_t* next_estimate, float dt) {
	if (!is_calibrated)
		return;
	
	LOCK_STATE_ESTIMATE;

//#ifdef USE_QUEST
//	quest_estimator_update(next_estimate->orientation_q);
//	//PRINT("Quest Orientation %f %f %f\n", next_estimate->orientation_euler[0] * 57.2957795131f, next_estimate->orientation_euler[1] * 57.2957795131f, next_estimate->orientation_euler[2] * 57.2957795131f);
//	//PRINT("QUEST: %f %f %f %f\n",next_estimate->orientation_q[0],next_estimate->orientation_q[1],next_estimate->orientation_q[2],next_estimate->orientation_q[3]);
//#else
//	mag_rotation_estimator_update(next_estimate->orientation_q);
//	//PRINT("Orientation %f %f %f\n", next_estimate->orientation_euler[0] * 57.2957795131f, next_estimate->orientation_euler[1] * 57.2957795131f, next_estimate->orientation_euler[2] * 57.2957795131f);
//#endif
//	
//#ifdef USE_ORIENTATION_KALMAN
//	orientation_kalman_new_quaternion(next_estimate->orientation_q);
//	orientation_kalman_prediction_step(next_estimate, dt);
//	//PRINT("Kalman: %f %f %f %f\n", next_estimate->orientation_q[0], next_estimate->orientation_q[1], next_estimate->orientation_q[2], next_estimate->orientation_q[3]);
//	//PRINT("Orientation %f %f %f\n", next_estimate->orientation_euler[0] * 57.2957795131f, next_estimate->orientation_euler[1] * 57.2957795131f, next_estimate->orientation_euler[2] * 57.2957795131f);
//#endif

#ifdef PRINT_ROTATED_ACCEL
	//TODO Rotate accel data by quaternion
	float rotatedAccel[3];
	float inverseQuat[4];
	inverseQuat[0] = -next_estimate->orientation_q[0];
	inverseQuat[1] = -next_estimate->orientation_q[1];
	inverseQuat[2] = -next_estimate->orientation_q[2];
	inverseQuat[3] = next_estimate->orientation_q[3];


	quat_rotate(next_estimate->orientation_q, accel_calibrated,rotatedAccel);
	PRINT("Rotated Accel %f %f %f\n", rotatedAccel[0], rotatedAccel[1], rotatedAccel[2]);

#endif

    
	//translation_kalman_prediction_step(next_estimate, dt);
	UNLOCK_STATE_ESTIMATE;
}