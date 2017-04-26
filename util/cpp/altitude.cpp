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


namespace plt = matplotlibcpp;


uint64_t timestamp = 0;
uint32_t last_timestamp = 0;

std::vector<float> mpu9250_altitudes;
std::vector<float> data_timestamps;
std::vector<float> ublox_altitudes;
std::vector<float> ublox_timestamps;
std::vector<float> ublox_error_bars;

std::vector<float> delta_timestamps;
std::vector<float> delta_altitudes;

static bool getPacket(const telemetry_t* packet, message_metadata_t metadata) {
    if (last_timestamp != 0) {
        timestamp += clocks_between(last_timestamp, packet->header.timestamp);
    }
    last_timestamp = packet->header.timestamp;

    if (packet->header.id == ts_ms5611_data) {
        auto data = telemetry_get_payload<ms5611data_t>(packet);
        auto altitude = ms5611_get_altitude(data);
        if (altitude == -9999.0f)
            return true;
        mpu9250_altitudes.push_back(altitude);
        if (ublox_altitudes.size() != 0) {
            delta_timestamps.push_back((float)timestamp / (float)platform_get_counter_frequency());
            delta_altitudes.push_back(std::abs(altitude - ublox_altitudes.back()));
        }

        data_timestamps.push_back((float)timestamp / (float)platform_get_counter_frequency());
    } else if (packet->header.id == ts_ublox_nav) {
        auto data = telemetry_get_payload<ublox_nav_t>(packet);
        if (data->fix_type != 3 || data->num_sv < 8 || (data->h_acc > 5000 && ublox_altitudes.empty()))
            return true;
        ublox_altitudes.push_back(data->height / 1000.0f);
        ublox_timestamps.push_back((float)timestamp / (float)platform_get_counter_frequency());
        ublox_error_bars.push_back(data->h_acc / 1000.0f);
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

    component_state_start(update_handler, false);
    messaging_all_start_options(false, false);

    messaging_consumer_init(&messaging_consumer);

    file_telemetry_input_start(input);

    // The file_telemetry_input can disconnect but there may still be packets in the consumers buffer from when we slept
    while (file_telemetry_input_connected() || messaging_consumer_receive(&messaging_consumer, false, false) == messaging_receive_ok) {
        // We have to "busy" wait as the consumer has no way to know when no further packets are going to arrive
        // A better solution would be to be able to specify a timeout but this hasn't been implemented
        while(messaging_consumer_receive(&messaging_consumer, false, false) == messaging_receive_ok);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    plt::errorbar(ublox_timestamps, ublox_altitudes, ublox_error_bars);
    plt::named_plot("MS5611 Altitudes", data_timestamps, mpu9250_altitudes);
    plt::named_plot("Altitude Delta", delta_timestamps, delta_altitudes);

    plt::grid(true);
    plt::legend();
    plt::save(output);
    return 0;
}
