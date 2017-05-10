#include "kalman_constants.h"

// The sensor noise covariance is given by
// - (Random Walk Value from Allan Deviation x sqrt(sample_rate))^2
// The bias process noise is given by
// - (Bias Instability from Allan Deviation)^2

// Sensor Covariances
fp kalman_magno_cov = 5.12e-5f;
fp kalman_accelerometer_cov = 1.49e-4f;
fp kalman_gyro_cov = 1.88e-4f;

// Model Covariances

fp initial_position_cov = 1e-4f;
fp position_process_noise = 1e-7f;

fp initial_velocity_cov = 1e-3f;
fp velocity_process_noise = 1e-7f;

fp initial_acceleration_cov = 1e-11f;
fp acceleration_process_noise = 1e0f;

fp initial_attitude_err_cov = 1e-11f;
fp attitude_err_process_noise = 1e-7f;

fp initial_angular_vel_cov = 1e-11f;
fp angular_vel_process_noise = 1e0f;

fp initial_gyro_bias_cov = 1e-10f;
fp gyro_bias_process_noise = 6.32e-10f;

fp initial_accel_bias_cov = 1e-7f;
fp accel_bias_process_noise = 1.22e-7f;

fp initial_magno_bias_cov = 1e-9f;
fp magno_bias_process_noise = 6.65e-9f;
