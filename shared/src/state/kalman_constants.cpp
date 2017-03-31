#include "kalman_constants.h"

// Sensor Covariances
fp kalman_magno_cov = 0.002f;
fp kalman_accelerometer_cov = 0.002f;
fp kalman_gyro_cov = 1e-2f;

// Model Covariances

const fp initial_position_cov = 1e-4f;
const fp position_process_noise = 1e-7f;

const fp initial_velocity_cov = 1e-3f;
const fp velocity_process_noise = 1e-7f;

const fp initial_acceleration_cov = 1e-8f;
fp acceleration_process_noise = 1e2f;

const fp initial_accel_bias_cov = 1e-1f;
fp accel_bias_process_noise = 1e-2f;

const fp initial_attitude_err_cov = 1e-2f;
fp attitude_err_process_noise = 1e-5f;

const fp initial_angular_vel_cov = 1e-5f;
fp angular_vel_process_noise = 1e1f;

const fp initial_gyro_bias_cov = 1e-6f;
fp gyro_bias_process_noise = 1e-3f;

const fp initial_magno_bias_cov = 1e-1f;
fp magno_bias_process_noise = 1e-5f;