#include <can_telemetry.h>
#include <usb_telemetry.h>
#include <thread>
#include <messaging_all.h>
#include <file_telemetry.h>
#include <zconf.h>
#include <cpp_utils.h>
#include <config/component_state_config.h>
#include <component_state.h>
#include <iostream>


void update_handler(avionics_component_t component, avionics_component_state_t state, int line) {
    if (state == state_error)
        fprintf(stderr, "Error in component %i with line %i\n", component, line);
}

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
    const char* input = nullptr;
    const char* output = nullptr;
    bool overwrite = false;

    for (int i = 1; i < argc; i++) {
        std::string option = std::string(argv[i]);
        if (option == "--input" || option == "-i") {
            if (i+1 == argc) {
                printf("Invalid Arguments\n");
                print_help();
                return 1;
            }
            input = argv[++i];
        } else if (option == "--output" || option == "-o") {
            if (i+1 == argc) {
                printf("Invalid Arguments\n");
                print_help();
                return 1;
            }
            input = argv[++i];
        } else if (option == "--help" || option == "-h") {
            print_help();
            return 0;
        } else if (option == "--stdout" || option == "-s") {
            output = "stdout.csv";
        } else if (option == "--replace" || option == "-r") {
            overwrite = true;
        } else {
            printf("Invalid Option %s\n", argv[i]);
            print_help();
            return 1;
        }
    }

    component_state_start(update_handler, true);
    messaging_all_start();
    file_telemetry_input_start(input);
    file_telemetry_output_start(output, overwrite);

    if (!file_telemetry_output_connected()) {
        fprintf(stderr, "No output specified\n");
        return 1;
    }

    int input_count = 0;
    if (usb_telemetry_connected())
        input_count++;
    if (can_telemetry_connected())
        input_count++;
    if (file_telemetry_input_connected())
        input_count++;

    if (input_count == 0) {
        fprintf(stderr, "No input source\n");
        return 1;
    } else if (input_count > 1) {
        fprintf(stderr, "Multiple input sources\n");
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
