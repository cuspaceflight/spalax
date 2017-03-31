#include <iostream>
#include <vector>
#include <config/telemetry_packets.h>
#include <time_utils.h>
#include <algorithm>
#include "Eigen/Core"
#include <Eigen/Geometry>
#include <calibration/mpu9250_calibration.h>
#include <state/state_estimate.h>
#include <state/kalman.h>
#include <state/math_util.h>
#include <file_telemetry.h>
#include <thread>
#include <chrono>


using namespace Eigen;


 static uint64_t mpu_timestamp = 0;
 static uint32_t last_mpu_timestamp = 0;

 static uint64_t state_timestamp = 0;
 static uint32_t last_state_timestamp = 0;

 static uint64_t state_debug_timestamp = 0;
 static uint32_t last_state_debug_timestamp = 0;

 std::vector<float> se_accel_x;
 std::vector<float> se_accel_y;
 std::vector<float> se_accel_z;
 std::vector<float> se_accel_norm;

 std::vector<float> se_rotation_x;
 std::vector<float> se_rotation_y;
 std::vector<float> se_rotation_z;

 std::vector<float> se_ang_velocity_x;
 std::vector<float> se_ang_velocity_y;
 std::vector<float> se_ang_velocity_z;
 std::vector<float> se_ang_vel_norm;

 std::vector<float> se_velocity_x;
 std::vector<float> se_velocity_y;
 std::vector<float> se_velocity_z;

 std::vector<float> accel_x;
 std::vector<float> accel_y;
 std::vector<float> accel_z;
 std::vector<float> accel_norm;

 std::vector<float> gyro_x;
 std::vector<float> gyro_y;
 std::vector<float> gyro_z;
 std::vector<float> gyro_norm;

 std::vector<float> magno_x;
 std::vector<float> magno_y;
 std::vector<float> magno_z;
 std::vector<float> magno_norm;

 std::vector<float> accel_magno_angle;
 std::vector<float> accel_magno_reference_angle;

 std::vector<float> se_gyro_bias_x;
 std::vector<float> se_gyro_bias_y;
 std::vector<float> se_gyro_bias_z;

 std::vector<float> se_accel_bias_x;
 std::vector<float> se_accel_bias_y;
 std::vector<float> se_accel_bias_z;
 std::vector<float> se_accel_bias_norm;

 std::vector<float> se_magno_bias_x;
 std::vector<float> se_magno_bias_y;
 std::vector<float> se_magno_bias_z;
 std::vector<float> se_magno_bias_norm;

 std::vector<float> mpu_timestamps;
 std::vector<float> state_timestamps;
 std::vector<float> state_debug_timestamps;

 bool getPacket(const telemetry_t *packet, message_metadata_t metadata) {
    if (packet->header.id == ts_mpu9250_data) {
        if (last_mpu_timestamp == 0) {
            last_mpu_timestamp = packet->header.timestamp;
            last_state_timestamp = packet->header.timestamp;
            last_state_debug_timestamp = packet->header.timestamp;
        }

        auto delta = clocks_between(last_mpu_timestamp, packet->header.timestamp);
        mpu_timestamp += delta;
        last_mpu_timestamp = packet->header.timestamp;

        auto data = telemetry_get_payload<mpu9250_data_t>(packet);
        mpu9250_calibrated_data_t calibrated_data;
        mpu9250_calibrate_data(data, &calibrated_data);

        mpu_timestamps.push_back((float) mpu_timestamp / (float) platform_get_counter_frequency());
        accel_x.push_back(calibrated_data.accel[0]);
        accel_y.push_back(calibrated_data.accel[1]);
        accel_z.push_back(calibrated_data.accel[2]);

        gyro_x.push_back(calibrated_data.gyro[0]);
        gyro_y.push_back(calibrated_data.gyro[1]);
        gyro_z.push_back(calibrated_data.gyro[2]);
        gyro_norm.push_back(Eigen::Map<const Matrix<fp, 3, 1>>(calibrated_data.gyro).norm());

        magno_x.push_back(calibrated_data.magno[0]);
        magno_y.push_back(calibrated_data.magno[1]);
        magno_z.push_back(calibrated_data.magno[2]);

        float angle = std::acos(Eigen::Map<const Matrix<fp, 3, 1>>(calibrated_data.accel).normalized().transpose() *
                                Eigen::Map<const Matrix<fp, 3, 1>>(calibrated_data.magno).normalized());

        accel_magno_angle.push_back(angle);

        magno_norm.push_back(Eigen::Map<const Matrix<fp, 3, 1>>(calibrated_data.magno).norm());
        accel_norm.push_back(Eigen::Map<const Matrix<fp, 3, 1>>(calibrated_data.accel).norm());

    } else if (packet->header.id == ts_state_estimate_data) {
        auto data = telemetry_get_payload<state_estimate_t>(packet);

        if (last_state_timestamp == 0) {
            return true;
        }

        state_timestamp += clocks_between(last_state_timestamp, packet->header.timestamp);
        last_state_timestamp = packet->header.timestamp;


        state_timestamps.push_back((float) state_timestamp / (float) platform_get_counter_frequency());
        se_accel_x.push_back(data->acceleration[0]);
        se_accel_y.push_back(data->acceleration[1]);
        se_accel_z.push_back(data->acceleration[2]);
        se_accel_norm.push_back(Eigen::Map<const Matrix<fp, 3, 1>>(data->acceleration).norm());


        Vector3f euler = 180.0f / 3.141592653589793f * quat_to_euler(
                Quaternionf(data->orientation_q[3], data->orientation_q[0], data->orientation_q[1],
                            data->orientation_q[2]));
        se_rotation_x.push_back(euler.x());
        se_rotation_y.push_back(euler.y());
        se_rotation_z.push_back(euler.z());

        se_velocity_x.push_back(data->velocity[0]);
        se_velocity_y.push_back(data->velocity[1]);
        se_velocity_z.push_back(data->velocity[2]);

        se_ang_velocity_x.push_back(data->angular_velocity[0]);
        se_ang_velocity_y.push_back(data->angular_velocity[1]);
        se_ang_velocity_z.push_back(data->angular_velocity[2]);

        se_ang_vel_norm.push_back(Eigen::Map<const Matrix<fp, 3, 1>>(data->angular_velocity).norm());
    } else if (packet->header.id == ts_state_estimate_debug) {
        auto data = telemetry_get_payload<state_estimate_debug_t>(packet);

        if (last_state_debug_timestamp == 0) {
            return true;
        }

        state_debug_timestamp += clocks_between(last_state_debug_timestamp, packet->header.timestamp);
        last_state_debug_timestamp = packet->header.timestamp;

        state_debug_timestamps.push_back((float) state_debug_timestamp / (float) platform_get_counter_frequency());

        se_gyro_bias_x.push_back(data->gyro_bias[0]);
        se_gyro_bias_y.push_back(data->gyro_bias[1]);
        se_gyro_bias_z.push_back(data->gyro_bias[2]);

        se_accel_bias_x.push_back(data->accel_bias[0]);
        se_accel_bias_y.push_back(data->accel_bias[1]);
        se_accel_bias_z.push_back(data->accel_bias[2]);

        se_accel_bias_norm.push_back(Eigen::Map<const Matrix<fp, 3, 1>>(data->accel_bias).norm());

        se_magno_bias_x.push_back(data->magno_bias[0]);
        se_magno_bias_y.push_back(data->magno_bias[1]);
        se_magno_bias_z.push_back(data->magno_bias[2]);

        se_magno_bias_norm.push_back(Eigen::Map<const Matrix<fp, 3, 1>>(data->magno_bias).norm());

        float angle = std::acos(Eigen::Map<const Matrix<fp, 3, 1>>(data->accel_ref).normalized().transpose() *
                                Eigen::Map<const Matrix<fp, 3, 1>>(data->magno_ref).normalized());

        accel_magno_reference_angle.push_back(angle);
    }


    return true;
}

MESSAGING_CONSUMER(messaging_consumer, ts_all, ts_all_mask, 0, 0, getPacket, 1024);

static void reset() {
    mpu_timestamp = 0;
    last_mpu_timestamp = 0;
    state_timestamp = 0;
    last_state_timestamp = 0;
    state_debug_timestamp = 0;
    last_state_debug_timestamp = 0;

    se_accel_x.clear();
    se_accel_y.clear();
    se_accel_z.clear();
    se_accel_norm.clear();

    se_rotation_x.clear();
    se_rotation_y.clear();
    se_rotation_z.clear();

    se_ang_velocity_x.clear();
    se_ang_velocity_y.clear();
    se_ang_velocity_z.clear();
    se_ang_vel_norm.clear();

    se_velocity_x.clear();
    se_velocity_y.clear();
    se_velocity_z.clear();

    accel_x.clear();
    accel_y.clear();
    accel_z.clear();
    accel_norm.clear();

    gyro_x.clear();
    gyro_y.clear();
    gyro_z.clear();
    gyro_norm.clear();

    magno_x.clear();
    magno_y.clear();
    magno_z.clear();
    magno_norm.clear();

    accel_magno_angle.clear();
    accel_magno_reference_angle.clear();

    se_gyro_bias_x.clear();
    se_gyro_bias_y.clear();
    se_gyro_bias_z.clear();

    se_accel_bias_x.clear();
    se_accel_bias_y.clear();
    se_accel_bias_z.clear();
    se_accel_bias_norm.clear();

    se_magno_bias_x.clear();
    se_magno_bias_y.clear();
    se_magno_bias_z.clear();
    se_magno_bias_norm.clear();

    mpu_timestamps.clear();
    state_timestamps.clear();
    state_debug_timestamps.clear();
}

void run_data_extractor(const char* input) {
    reset();

    messaging_consumer_init(&messaging_consumer);

    state_estimate_init();

    file_telemetry_input_start(input);

    std::thread state_estimate(state_estimate_thread, nullptr);

    // The file_telemetry_input can disconnect but there may still be packets in the consumers buffer from when we slept
    while (file_telemetry_input_connected() ||
           messaging_consumer_receive(&messaging_consumer, false, false) == messaging_receive_ok) {
        // We have to "busy" wait as the consumer has no way to know when no further packets are going to arrive
        // A better solution would be to be able to specify a timeout but this hasn't been implemented
        while (messaging_consumer_receive(&messaging_consumer, false, false) == messaging_receive_ok);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    state_estimate_terminate();

    state_estimate.join();

    file_telemetry_input_stop();

    messaging_consumer_terminate(&messaging_consumer);
}

