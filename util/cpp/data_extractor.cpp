#include <iostream>
#include <vector>
#include <config/telemetry_packets.h>
#include <time_utils.h>
#include <algorithm>
#include "Eigen/Core"
#include "data_extractor.h"
#include <Eigen/Geometry>
#include <calibration/mpu9250_calibration.h>
#include <state/state_estimate.h>
#include <state/kalman.h>
#include <state/math_util.h>
#include <file_telemetry.h>
#include <thread>


using namespace Eigen;


static uint64_t mpu_timestamp = 0;
static uint32_t last_mpu_timestamp = 0;

static uint64_t state_timestamp = 0;
static uint32_t last_state_timestamp = 0;

static uint64_t state_debug_timestamp = 0;
static uint32_t last_state_debug_timestamp = 0;

static DataExtractor *de = nullptr;

bool getPacket(const telemetry_t *packet, message_metadata_t metadata) {
    (void) metadata;

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

        de->mpu_timestamps.push_back((float) mpu_timestamp / (float) platform_get_counter_frequency());
        de->accel_x.push_back(calibrated_data.accel[0]);
        de->accel_y.push_back(calibrated_data.accel[1]);
        de->accel_z.push_back(calibrated_data.accel[2]);

        de->gyro_x.push_back(calibrated_data.gyro[0]);
        de->gyro_y.push_back(calibrated_data.gyro[1]);
        de->gyro_z.push_back(calibrated_data.gyro[2]);
        de->gyro_norm.push_back(Eigen::Map<const Matrix<fp, 3, 1>>(calibrated_data.gyro).norm());

        de->magno_x.push_back(calibrated_data.magno[0]);
        de->magno_y.push_back(calibrated_data.magno[1]);
        de->magno_z.push_back(calibrated_data.magno[2]);

        float angle = std::acos(Eigen::Map<const Matrix<fp, 3, 1>>(calibrated_data.accel).normalized().transpose() *
                                Eigen::Map<const Matrix<fp, 3, 1>>(calibrated_data.magno).normalized());

        de->accel_magno_angle.push_back(angle);

        de->magno_norm.push_back(Eigen::Map<const Matrix<fp, 3, 1>>(calibrated_data.magno).norm());
        de->accel_norm.push_back(Eigen::Map<const Matrix<fp, 3, 1>>(calibrated_data.accel).norm());

    } else if (packet->header.id == ts_state_estimate_data) {
        auto data = telemetry_get_payload<state_estimate_t>(packet);

        if (last_state_timestamp == 0) {
            return true;
        }

        state_timestamp += clocks_between(last_state_timestamp, packet->header.timestamp);
        last_state_timestamp = packet->header.timestamp;


        de->state_timestamps.push_back((float) state_timestamp / (float) platform_get_counter_frequency());
        de->se_accel_x.push_back(data->acceleration[0]);
        de->se_accel_y.push_back(data->acceleration[1]);
        de->se_accel_z.push_back(data->acceleration[2]);
        de->se_accel_norm.push_back(Eigen::Map<const Matrix<fp, 3, 1>>(data->acceleration).norm());


        Vector3f euler = 180.0f / 3.141592653589793f * quat_to_euler(
                Quaternionf(data->orientation_q[3], data->orientation_q[0], data->orientation_q[1],
                            data->orientation_q[2]));
        de->se_rotation_x.push_back(euler.x());
        de->se_rotation_y.push_back(euler.y());
        de->se_rotation_z.push_back(euler.z());

        de->se_velocity_x.push_back(data->velocity[0]);
        de->se_velocity_y.push_back(data->velocity[1]);
        de->se_velocity_z.push_back(data->velocity[2]);

        de->se_ang_velocity_x.push_back(data->angular_velocity[0]);
        de->se_ang_velocity_y.push_back(data->angular_velocity[1]);
        de->se_ang_velocity_z.push_back(data->angular_velocity[2]);

        de->se_ang_vel_norm.push_back(Eigen::Map<const Matrix<fp, 3, 1>>(data->angular_velocity).norm());
    } else if (packet->header.id == ts_state_estimate_debug) {
        auto data = telemetry_get_payload<state_estimate_debug_t>(packet);

        if (last_state_debug_timestamp == 0) {
            return true;
        }

        state_debug_timestamp += clocks_between(last_state_debug_timestamp, packet->header.timestamp);
        last_state_debug_timestamp = packet->header.timestamp;

        de->state_debug_timestamps.push_back((float) state_debug_timestamp / (float) platform_get_counter_frequency());

        de->se_gyro_bias_x.push_back(data->gyro_bias[0]);
        de->se_gyro_bias_y.push_back(data->gyro_bias[1]);
        de->se_gyro_bias_z.push_back(data->gyro_bias[2]);

        de->se_accel_bias_x.push_back(data->accel_bias[0]);
        de->se_accel_bias_y.push_back(data->accel_bias[1]);
        de->se_accel_bias_z.push_back(data->accel_bias[2]);

        de->se_accel_bias_norm.push_back(Eigen::Map<const Matrix<fp, 3, 1>>(data->accel_bias).norm());

        de->se_magno_bias_x.push_back(data->magno_bias[0]);
        de->se_magno_bias_y.push_back(data->magno_bias[1]);
        de->se_magno_bias_z.push_back(data->magno_bias[2]);

        de->se_magno_bias_norm.push_back(Eigen::Map<const Matrix<fp, 3, 1>>(data->magno_bias).norm());

        float angle = std::acos(Eigen::Map<const Matrix<fp, 3, 1>>(data->accel_ref).normalized().transpose() *
                                Eigen::Map<const Matrix<fp, 3, 1>>(data->magno_ref).normalized());

        de->accel_magno_reference_angle.push_back(angle);
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
}

void run_data_extractor(const char *input, DataExtractor *extractor) {
    assert(de == nullptr);

    de = extractor;

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

    de = nullptr;
}

