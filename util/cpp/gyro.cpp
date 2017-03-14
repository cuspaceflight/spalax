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


namespace plt = matplotlibcpp;
using namespace Eigen;



uint64_t mpu_timestamp = 0;
uint32_t last_mpu_timestamp = 0;

uint64_t state_timestamp = 0;
uint32_t last_state_timestamp = 0;

std::vector<float> vel_x;
std::vector<float> vel_y;
std::vector<float> vel_z;

std::vector<float> gyro_x;
std::vector<float> gyro_y;
std::vector<float> gyro_z;

std::vector<float> gyro_bias_x;
std::vector<float> gyro_bias_y;
std::vector<float> gyro_bias_z;

std::vector<float> mpu_timestamps;
std::vector<float> state_timestamps;

static bool getPacket(const telemetry_t* packet, message_metadata_t metadata) {
    if (packet->header.id == ts_mpu9250_data) {
        if (last_mpu_timestamp == 0) {
            last_mpu_timestamp = packet->header.timestamp;
            last_state_timestamp = packet->header.timestamp;
        }

        auto delta = clocks_between(last_mpu_timestamp, packet->header.timestamp);
        mpu_timestamp += delta;
        last_mpu_timestamp = packet->header.timestamp;

        auto data = telemetry_get_payload<mpu9250_data_t>(packet);
        mpu9250_calibrated_data_t calibrated_data;
        mpu9250_calibrate_data(data, &calibrated_data);

        mpu_timestamps.push_back((float) mpu_timestamp / (float) platform_get_counter_frequency());
        gyro_x.push_back(calibrated_data.gyro[0]);
        gyro_y.push_back(calibrated_data.gyro[1]);
        gyro_z.push_back(calibrated_data.gyro[2]);
    }
    else if (packet->header.id == ts_state_estimate_data) {
        auto data = telemetry_get_payload<state_estimate_t>(packet);

        if (last_state_timestamp == 0) {
            return true;
        }

        state_timestamp += clocks_between(last_state_timestamp, data->data_timestamp);
        last_state_timestamp = data->data_timestamp;


        state_timestamps.push_back((float)state_timestamp / (float)platform_get_counter_frequency());
        vel_x.push_back(data->angular_velocity[0]);
        vel_y.push_back(data->angular_velocity[1]);
        vel_z.push_back(data->angular_velocity[2]);

        float bias[3];

        kalman_get_gyro_bias(bias);
        gyro_bias_x.push_back(bias[0]);
        gyro_bias_y.push_back(bias[1]);
        gyro_bias_z.push_back(bias[2]);

    }



    return true;
}

MESSAGING_CONSUMER(messaging_consumer, ts_all, ts_all_mask, 0, 0, getPacket, 1024);

void update_handler(avionics_component_t component, avionics_component_state_t state, int line) {
    if (state == state_error)
        fprintf(stderr, "Error in component %i with line %i\n", component, line);
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Invalid arguments - expected <Input> <Output>";
        return 1;
    }
    const char* input = argv[1];
    const char* output = argv[2];

    setBoardConfig(BoardConfigSpalax);

    component_state_start(update_handler, false);
    messaging_all_start_options(false, false);

    messaging_consumer_init(&messaging_consumer);

    file_telemetry_input_start(input);

    std::thread state_estimate(state_estimate_thread, nullptr);

    // The file_telemetry_input can disconnect but there may still be packets in the consumers buffer from when we slept
    while (file_telemetry_input_connected() || messaging_consumer_receive(&messaging_consumer, false, false) == messaging_receive_ok) {
        // We have to "busy" wait as the consumer has no way to know when no further packets are going to arrive
        // A better solution would be to be able to specify a timeout but this hasn't been implemented
        while(messaging_consumer_receive(&messaging_consumer, false, false) == messaging_receive_ok);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    state_estimate_terminate();

    state_estimate.join();

    plt::named_plot("Gyro X", mpu_timestamps, gyro_x);
    //plt::named_plot("Gyro Y", mpu_timestamps, gyro_y);
    //plt::named_plot("Gyro Z", mpu_timestamps, gyro_z);

    plt::named_plot("Vel X", state_timestamps, vel_x);


    //plt::named_plot("Vel Y", mpu_timestamps, vel_y);
    //plt::named_plot("Vel Z", mpu_timestamps, vel_z);

    plt::named_plot("Gyro Bias X", state_timestamps, gyro_bias_x);

    plt::grid(true);
    plt::legend();
    plt::save(output);
    return 0;
}
