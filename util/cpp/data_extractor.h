#pragma once
#include <vector>

struct DataItem {
    bool enabled;
    std::vector<float> data;

    DataItem() : enabled(true) {

    }

    void push_back(float t) {
        if (enabled)
            data.push_back(t);
    }

    std::vector<float>::iterator begin() {
        return data.begin();
    }

    std::vector<float>::iterator end() {
        return data.end();
    }
};

struct DataExtractor {
    DataItem se_accel_x;
    DataItem se_accel_y;
    DataItem se_accel_z;
    DataItem se_accel_norm;

    DataItem se_rotation_x;
    DataItem se_rotation_y;
    DataItem se_rotation_z;

    DataItem se_ang_velocity_x;
    DataItem se_ang_velocity_y;
    DataItem se_ang_velocity_z;
    DataItem se_ang_vel_norm;

    DataItem se_velocity_x;
    DataItem se_velocity_y;
    DataItem se_velocity_z;

    DataItem accel_x;
    DataItem accel_y;
    DataItem accel_z;
    DataItem accel_norm;

    DataItem gyro_x;
    DataItem gyro_y;
    DataItem gyro_z;
    DataItem gyro_norm;

    DataItem magno_x;
    DataItem magno_y;
    DataItem magno_z;
    DataItem magno_norm;

    DataItem accel_magno_angle;
    DataItem accel_magno_reference_angle;

    DataItem se_gyro_bias_x;
    DataItem se_gyro_bias_y;
    DataItem se_gyro_bias_z;

    DataItem se_accel_bias_x;
    DataItem se_accel_bias_y;
    DataItem se_accel_bias_z;
    DataItem se_accel_bias_norm;

    DataItem se_magno_bias_x;
    DataItem se_magno_bias_y;
    DataItem se_magno_bias_z;
    DataItem se_magno_bias_norm;

    std::vector<float> mpu_timestamps;
    std::vector<float> state_timestamps;
    std::vector<float> state_debug_timestamps;
};

void run_data_extractor(const char* input, DataExtractor* extractor);