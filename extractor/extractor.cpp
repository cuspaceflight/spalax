#include "avionics_config.h"
#include <messaging.h>
#include <checksum.h>
#include <component_state.h>
#include <can_interface.h>
#include "telemetry_packets.h"
#include <fstream>
#include <atomic>
#include <iostream>
#include <cstring>
#include <can_telemetry.h>
#include <usb_telemetry.h>
#include <thread>
#include <messaging_all.h>

void update_handler(avionics_component_t component, avionics_component_state_t state, int line) {
    if (state == state_error)
        printf("Error in component %i with line %i\n", component, line);
}

const avionics_config_t local_config = {telemetry_origin_avionics_gui, update_handler };

std::ofstream* mpu_stream;
std::atomic<bool> running(true);

static bool mpu_consumer_func(const telemetry_t* packet, message_metadata_t metadata) {
    if (packet->header.id == telemetry_id_mpu9250_data) {
        mpu9250_data_t* data = (mpu9250_data_t*)packet->payload;

        *mpu_stream << data->accel[0] << ',';
        *mpu_stream << data->accel[1] << ',';
        *mpu_stream << data->accel[2] << ',';

        *mpu_stream << data->gyro[0] << ',';
        *mpu_stream << data->gyro[1] << ',';
        *mpu_stream << data->gyro[2] << ',';

        *mpu_stream << data->magno[0] << ',';
        *mpu_stream << data->magno[1] << ',';
        *mpu_stream << data->magno[2] << std::endl;
    }
    return true;
}


MESSAGING_CONSUMER(mpu_consumer, telemetry_source_mpu9250, telemetry_source_mpu9250_mask, 0, 0, mpu_consumer_func, 1024);

void rocket_main() {
    messaging_all_start();

    messaging_consumer_init(&mpu_consumer);

    if (!can_telemetry_connected() && !usb_telemetry_connected()) {
        printf("No valid data source found\n");
        exit(1);
    }

    while(running) {
        messaging_consumer_receive(&mpu_consumer, true, false);
    }
}

int main() {
    auto mpu_file_name = "mpu_data.csv";
    printf("Writing MPU data to %s\n", mpu_file_name);

    mpu_stream = new std::ofstream(mpu_file_name);

    std::thread rocket_thread(rocket_main);

    char line[256];
    while (true) {
        std::cin.getline(line, 256);
        if (strcmp(line, "quit") == 0)
            break;
    }

    running = false;
    messaging_consumer_terminate(&mpu_consumer);

    rocket_thread.join();

    mpu_stream->close();
    delete mpu_stream;
    mpu_stream = nullptr;

    return 0;
}
