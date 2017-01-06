#include "avionics_config.h"
#include <messaging.h>
#include "telemetry_packets.h"
#include <fstream>
#include <atomic>
#include <iostream>
#include <cstring>
#include <can_telemetry.h>
#include <usb_telemetry.h>
#include <thread>
#include <messaging_all.h>
#include <file_telemetry.h>
#include <zconf.h>
#include <cpp_utils.h>


void update_handler(avionics_component_t component, avionics_component_state_t state, int line) {
    if (state == state_error)
        printf("Error in component %i with line %i\n", component, line);
}

avionics_config_t local_config = {update_handler, nullptr, nullptr, false, true};

int wait_for_input(int seconds) {
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = seconds;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}

static void print_help() {
    printf("-h, --help                 Print this help\n");
    printf("-i, --input                Specify input file - the format will be determined by the extension\n");
    printf("-o, --output               Specify output file - the format will be determined by the extension\n");
    printf("-r, --replace              Overwrite existing files\n");
    printf("-s, --stdout               Print CSV output to stdout\n");
}

int main(int argc, char* argv[]) {
    for (int i = 1; i < argc; i++) {
        std::string option = std::string(argv[i]);
        if (option == "--input" || option == "-i") {
            if (i+1 == argc) {
                printf("Invalid Arguments\n");
                print_help();
                return 1;
            }
            local_config.input_file_name = argv[++i];
        } else if (option == "--output" || option == "-o") {
            if (i+1 == argc) {
                printf("Invalid Arguments\n");
                print_help();
                return 1;
            }
            local_config.output_file_name = argv[++i];
        } else if (option == "--help" || option == "-h") {
            print_help();
            return 0;
        } else if (option == "--stdout" || option == "-s") {
            local_config.output_file_name = "stdout.csv";
        } else if (option == "--replace" || option == "-r") {
            local_config.output_file_overwrite_enabled = true;
        } else {
            printf("Invalid Option %s\n", argv[i]);
            print_help();
            return 1;
        }
    }

    messaging_all_start();

    if (local_config.output_file_name && !file_telemetry_output_connected()) {
        return 1;
    }

    while (usb_telemetry_connected() || can_telemetry_connected() || file_telemetry_input_connected()) {
        if (wait_for_input(1) != 0) {
            std::string str;
            std::cin >> str;
            if (str == "quit")
                break;
        }
    }

    return 0;
}
