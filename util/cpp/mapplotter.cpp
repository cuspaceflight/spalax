

#include <messaging_all.h>
#include <file_telemetry.h>
#include <cpp_utils.h>
#include <config/component_state_config.h>
#include <component_state.h>
#include <iostream>
#include <vector>
#include <config/telemetry_packets.h>
#include <chrono>
#include <thread>
#include <algorithm>
#include <util/board_config.h>
#include <wrappy/wrappy.h>
#include <time_utils.h>

std::vector<float> latitudes;
std::vector<float> longitudes;

std::vector<float> pin_latitudes;
std::vector<float> pin_longitudes;

float next_interval = 0;

uint64_t timestamp = 0;
uint32_t last_timestamp = 0;

static bool getPacket(const telemetry_t* packet, message_metadata_t metadata) {
    if (last_timestamp != 0) {
        timestamp += clocks_between(last_timestamp, packet->header.timestamp);
    }
    last_timestamp = packet->header.timestamp;

    if (packet->header.id == ts_ublox_nav) {
        auto data = telemetry_get_payload<ublox_nav_t>(packet);
        if (data->fix_type != 3 || data->num_sv < 8)
            return true;
        latitudes.push_back(data->lat / 10000000.0f);
        longitudes.push_back(data->lon / 10000000.0f);
        if ((float)timestamp / (float)platform_get_counter_frequency() > next_interval) {
            pin_latitudes.push_back(data->lat / 10000000.0f);
            pin_longitudes.push_back(data->lon / 10000000.0f);
            next_interval += 100.0f;
        }

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

    setBoardConfig(BoardConfigSpalaxBrokenSD);

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

    std::vector<wrappy::PythonObject> py_latitudes;
    std::vector<wrappy::PythonObject> py_longitudes;

    std::transform(latitudes.begin(), latitudes.end(), std::back_inserter(py_latitudes),
                   [](float d) { return wrappy::construct(d); });

    std::transform(longitudes.begin(), longitudes.end(), std::back_inserter(py_longitudes),
                   [](float d) { return wrappy::construct(d); });

    auto gmap = wrappy::call("gmplot.GoogleMapPlotter", *latitudes.begin(), *longitudes.begin(), 16);

    gmap.call("plot", py_latitudes, py_longitudes, "cornflowerblue");


    for (int i = 0; i < std::min(pin_latitudes.size(), pin_longitudes.size()); i++) {
        gmap.call("marker", pin_latitudes[i], pin_longitudes[i]);
    }

    gmap.call("draw", output);

    return 0;
}
