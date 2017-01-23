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

std::vector<float> ms5611_altitudes;
std::vector<float> ms5611_timestamps;
std::vector<float> ublox_altitudes;
std::vector<float> ublox_timestamps;

static bool getPacket(const telemetry_t* packet, message_metadata_t metadata) {
    if (last_timestamp != 0) {
        timestamp += clocks_between(last_timestamp, packet->header.timestamp);
    }
    last_timestamp = packet->header.timestamp;

    if (packet->header.id == ts_ms5611_data) {
        auto data = telemetry_get_payload<ms5611data_t>(packet);
        auto altitude = ms5611_get_altitude(data->pressure);
        if (altitude == -9999.0f)
            return true;
        ms5611_altitudes.push_back(altitude);
        ms5611_timestamps.push_back((float)timestamp / (float)platform_get_counter_frequency());
    } else if (packet->header.id == ts_ublox_nav) {
        auto data = telemetry_get_payload<ublox_nav_t>(packet);
        if (data->fix_type != 3 && data->num_sv < 4)
            return true;
        // Fudge number chosen to put lines close together
        ublox_altitudes.push_back(data->height / 1000.0f - 127.890396f);
        ublox_timestamps.push_back((float)timestamp / (float)platform_get_counter_frequency());
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

    plt::named_plot("Ublox Altitudes", ublox_timestamps, ublox_altitudes);
    plt::named_plot("MS5611 Altitudes", ms5611_timestamps, ms5611_altitudes);

    plt::legend();
    plt::save(output);
    return 0;
}
