#include <messaging_all.h>
#include <file_telemetry.h>
#include <cpp_utils.h>
#include <config/component_state_config.h>
#include <component_state.h>
#include <iostream>
#include <vector>
#include <config/telemetry_packets.h>
#include <calibration/ms5611_calibration.h>
#include <time_utils.h>
#include <matplotlibcpp.h>
#include <chrono>
#include <thread>
#include <algorithm>
#include "Eigen/Core"
#include <Eigen/Geometry>
#include <state/quest.h>
#include <calibration/mpu9250_calibration.h>
#include <util/board_config.h>
#include <state/state_estimate.h>
#include <state/kalman.h>
#include <state/math_util.h>


namespace plt = matplotlibcpp;
using namespace Eigen;


static uint64_t mpu_timestamp = 0;
static uint32_t last_mpu_timestamp = 0;

static uint64_t state_timestamp = 0;
static uint32_t last_state_timestamp = 0;

static uint64_t state_debug_timestamp = 0;
static uint32_t last_state_debug_timestamp = 0;

static std::vector<float> se_accel_x;
static std::vector<float> se_accel_y;
static std::vector<float> se_accel_z;
static std::vector<float> se_accel_norm;

static std::vector<float> se_rotation_x;
static std::vector<float> se_rotation_y;
static std::vector<float> se_rotation_z;

static std::vector<float> se_ang_velocity_x;
static std::vector<float> se_ang_velocity_y;
static std::vector<float> se_ang_velocity_z;
static std::vector<float> se_ang_vel_norm;

static std::vector<float> se_velocity_x;
static std::vector<float> se_velocity_y;
static std::vector<float> se_velocity_z;

static std::vector<float> accel_x;
static std::vector<float> accel_y;
static std::vector<float> accel_z;
static std::vector<float> accel_norm;

static std::vector<float> gyro_x;
static std::vector<float> gyro_y;
static std::vector<float> gyro_z;

static std::vector<float> magno_x;
static std::vector<float> magno_y;
static std::vector<float> magno_z;
static std::vector<float> magno_norm;

static std::vector<float> accel_magno_angle;

static std::vector<float> se_gyro_bias_x;
static std::vector<float> se_gyro_bias_y;
static std::vector<float> se_gyro_bias_z;

static std::vector<float> se_accel_bias_x;
static std::vector<float> se_accel_bias_y;
static std::vector<float> se_accel_bias_z;
static std::vector<float> se_accel_bias_norm;

static std::vector<float> se_magno_bias_x;
static std::vector<float> se_magno_bias_y;
static std::vector<float> se_magno_bias_z;
static std::vector<float> se_magno_bias_norm;

static std::vector<float> mpu_timestamps;
static std::vector<float> state_timestamps;
static std::vector<float> state_debug_timestamps;

static bool getPacket(const telemetry_t *packet, message_metadata_t metadata) {
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


        Vector3f euler = quat_to_euler(
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
    } else if (packet->header.id == ts_state_estimate_debug_data) {
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
    }


    return true;
}

MESSAGING_CONSUMER(messaging_consumer, ts_all, ts_all_mask, 0, 0, getPacket, 1024);

void update_handler(avionics_component_t component, avionics_component_state_t state, int line) {
    if (state == state_error)
        fprintf(stderr, "Error in component %i with line %i\n", component, line);
}

int main(int argc, char *argv[]) {
    if (argc < 4 || argc % 2 != 0) {
        std::cerr << "Invalid arguments - expected <Input> [<Output> <Format>]...";
        return 1;
    }
    const char *input = argv[1];


    setBoardConfig(BoardConfigSpalax);

    component_state_start(update_handler, false);
    messaging_all_start_options(false, false);

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

    for (int k = 2; k < argc; k+= 2) {
        const char *output = argv[k];
        std::istringstream f(argv[k+1]);
        std::string graph_variable;

        while (getline(f, graph_variable, ';')) {
            if (graph_variable == "AX")
                plt::named_plot("Accel X", mpu_timestamps, accel_x);
            else if (graph_variable == "AY")
                plt::named_plot("Accel Y", mpu_timestamps, accel_y);
            else if (graph_variable == "AZ")
                plt::named_plot("Accel Z", mpu_timestamps, accel_z);

            else if (graph_variable == "GX")
                plt::named_plot("Gyro X", mpu_timestamps, gyro_x);
            else if (graph_variable == "GY")
                plt::named_plot("Gyro Y", mpu_timestamps, gyro_y);
            else if (graph_variable == "GZ")
                plt::named_plot("Gyro Z", mpu_timestamps, gyro_z);

            else if (graph_variable == "MX")
                plt::named_plot("Magno X", mpu_timestamps, magno_x);
            else if (graph_variable == "MY")
                plt::named_plot("Magno Y", mpu_timestamps, magno_y);
            else if (graph_variable == "MZ")
                plt::named_plot("Magno Z", mpu_timestamps, magno_z);

            else if (graph_variable == "AMANGLE")
                plt::named_plot("Accel Magno Angle", mpu_timestamps, accel_magno_angle);
            else if (graph_variable == "MNORM")
                plt::named_plot("Magno Normal", mpu_timestamps, magno_norm);
            else if (graph_variable == "ANORM")
                plt::named_plot("Accel Magnitude", mpu_timestamps, accel_norm);

            else if (graph_variable == "SEAX")
                plt::named_plot("SE Accel X", state_timestamps, se_accel_x);
            else if (graph_variable == "SEAY")
                plt::named_plot("SE Accel Y", state_timestamps, se_accel_y);
            else if (graph_variable == "SEAZ")
                plt::named_plot("SE Accel Z", state_timestamps, se_accel_z);
            else if (graph_variable == "SEANORM")
                plt::named_plot("SE Accel Magnitude", state_timestamps, se_accel_norm);

            else if (graph_variable == "SEVX")
                plt::named_plot("SE Velocity X", state_timestamps, se_velocity_x);
            else if (graph_variable == "SEVY")
                plt::named_plot("SE Velocity Y", state_timestamps, se_velocity_y);
            else if (graph_variable == "SEVZ")
                plt::named_plot("SE Velocity Z", state_timestamps, se_velocity_z);

            else if (graph_variable == "SERX")
                plt::named_plot("SE Rotation X", state_timestamps, se_rotation_x);
            else if (graph_variable == "SERY")
                plt::named_plot("SE Rotation Y", state_timestamps, se_rotation_y);
            else if (graph_variable == "SERZ")
                plt::named_plot("SE Rotation Z", state_timestamps, se_rotation_z);

            else if (graph_variable == "SEAVX")
                plt::named_plot("SE Angular Velocity X", state_timestamps, se_ang_velocity_x);
            else if (graph_variable == "SEAVY")
                plt::named_plot("SE Angular Velocity Y", state_timestamps, se_ang_velocity_y);
            else if (graph_variable == "SEAVZ")
                plt::named_plot("SE Angular Velocity Z", state_timestamps, se_ang_velocity_z);
            else if (graph_variable == "SEAV")
                plt::named_plot("SE Angular Velocity Magnitude", state_timestamps, se_ang_vel_norm);

            else if (graph_variable == "SEGBX")
                plt::named_plot("SE Gyro Bias X", state_debug_timestamps, se_gyro_bias_x);
            else if (graph_variable == "SEGBY")
                plt::named_plot("SE Gyro Bias Y", state_debug_timestamps, se_gyro_bias_y);
            else if (graph_variable == "SEGBZ")
                plt::named_plot("SE Gyro Bias Z", state_debug_timestamps, se_gyro_bias_z);

            else if (graph_variable == "SEABX")
                plt::named_plot("SE Accel Bias X", state_debug_timestamps, se_accel_bias_x);
            else if (graph_variable == "SEABY")
                plt::named_plot("SE Accel Bias Y", state_debug_timestamps, se_accel_bias_y);
            else if (graph_variable == "SEABZ")
                plt::named_plot("SE Accel Bias Z", state_debug_timestamps, se_accel_bias_z);
            else if (graph_variable == "SEABNORM")
                plt::named_plot("SE Accel Bias Magnitude", state_debug_timestamps, se_accel_bias_norm);

            else if (graph_variable == "SEMBX")
                plt::named_plot("SE Magno Bias X", state_debug_timestamps, se_magno_bias_x);
            else if (graph_variable == "SEMBY")
                plt::named_plot("SE Magno Bias Y", state_debug_timestamps, se_magno_bias_y);
            else if (graph_variable == "SEMBZ")
                plt::named_plot("SE Magno Bias Z", state_debug_timestamps, se_magno_bias_z);
            else if (graph_variable == "SEMBNORM")
                plt::named_plot("SE Magno Bias Magnitude", state_debug_timestamps, se_magno_bias_norm);

        }

        plt::grid(true);
        plt::legend();
        plt::save(output);
        plt::clf();
    }
    return 0;
}
