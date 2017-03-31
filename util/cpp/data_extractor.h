#pragma once
#include <vector>


extern std::vector<float> se_accel_x;
extern std::vector<float> se_accel_y;
extern std::vector<float> se_accel_z;
extern std::vector<float> se_accel_norm;

extern std::vector<float> se_rotation_x;
extern std::vector<float> se_rotation_y;
extern std::vector<float> se_rotation_z;

extern std::vector<float> se_ang_velocity_x;
extern std::vector<float> se_ang_velocity_y;
extern std::vector<float> se_ang_velocity_z;
extern std::vector<float> se_ang_vel_norm;

extern std::vector<float> se_velocity_x;
extern std::vector<float> se_velocity_y;
extern std::vector<float> se_velocity_z;

extern std::vector<float> accel_x;
extern std::vector<float> accel_y;
extern std::vector<float> accel_z;
extern std::vector<float> accel_norm;

extern std::vector<float> gyro_x;
extern std::vector<float> gyro_y;
extern std::vector<float> gyro_z;
extern std::vector<float> gyro_norm;

extern std::vector<float> magno_x;
extern std::vector<float> magno_y;
extern std::vector<float> magno_z;
extern std::vector<float> magno_norm;

extern std::vector<float> accel_magno_angle;
extern std::vector<float> accel_magno_reference_angle;

extern std::vector<float> se_gyro_bias_x;
extern std::vector<float> se_gyro_bias_y;
extern std::vector<float> se_gyro_bias_z;

extern std::vector<float> se_accel_bias_x;
extern std::vector<float> se_accel_bias_y;
extern std::vector<float> se_accel_bias_z;
extern std::vector<float> se_accel_bias_norm;

extern std::vector<float> se_magno_bias_x;
extern std::vector<float> se_magno_bias_y;
extern std::vector<float> se_magno_bias_z;
extern std::vector<float> se_magno_bias_norm;

extern std::vector<float> mpu_timestamps;
extern std::vector<float> state_timestamps;
extern std::vector<float> state_debug_timestamps;

void run_data_extractor(const char* input);