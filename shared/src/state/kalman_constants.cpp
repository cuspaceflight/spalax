#include "kalman_constants.h"

// Sensor Covariances
fp kalman_magno_cov = 3.16e-4f;
fp kalman_accelerometer_cov = 4.9e-3f;
fp kalman_gyro_cov = 2.8e-4f;

// Model Covariances

fp initial_position_cov = 1e-4f;
fp position_process_noise = 1e-7f;

fp initial_velocity_cov = 1e-3f;
fp velocity_process_noise = 1e-7f;

fp initial_acceleration_cov = 1e-7f;
fp acceleration_process_noise = 1e0f;

fp initial_attitude_err_cov = 1e-5f;
fp attitude_err_process_noise = 1e-7f;

fp initial_angular_vel_cov = 1e-4f;
fp angular_vel_process_noise = 1e0f;

fp initial_gyro_bias_cov = 1e-5f;
fp gyro_bias_process_noise = 8.74e-5f;

fp initial_accel_bias_cov = 1e-5f;
fp accel_bias_process_noise = 2.5e-3f;

fp initial_magno_bias_cov = 1e-5f;
fp magno_bias_process_noise = 1e-4f;
