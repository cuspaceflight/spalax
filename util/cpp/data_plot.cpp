#include <messaging_all.h>
#include <cpp_utils.h>
#include <config/component_state_config.h>
#include <component_state.h>
#include <matplotlibcpp.h>
#include <util/board_config.h>
#include "data_extractor.h"
#include "data_plotter.h"


namespace plt = matplotlibcpp;

void update_handler(avionics_component_t component, avionics_component_state_t state, int line) {
    if (state == state_error)
        fprintf(stderr, "Error in component %i with line %i\n", component, line);
}

static void print_help() {
    std::cout << "Usage: data_plot [Options]... <Input> [<Output> <Variable>[:<Filter Options>][;[<Variable>[:<Filter Options>]]]...]..." << std::endl;

    std::cout << std::endl;
    std::cout << "Options" << std::endl;

    printf("-h, --help                 Print this help\n");
    printf("-s, --state                Run state estimators\n");

    std::cout << std::endl;
    std::cout << "Filter Options" << std::endl;
    printf("av<N>                      Plot N element average\n");
    printf("of<N>                      Plot values offset by N\n");
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cerr << "Too few arguments";
        print_help();
        return 1;
    }
    argv++;
    argc--;

    setBoardConfig(BoardConfigSpalax);

    component_state_start(update_handler, false);
    messaging_all_start_options(false, false);

    bool run_state_estimators = false;

    while (true) {
        std::string arg(argv[0]);
        if (arg[0] != '-')
            break;

        if (arg == "-h" || arg == "--help") {
            print_help();
            return 0;
        } else if (arg == "-s" || arg == "--state") {
            run_state_estimators = true;
        }

        argv++;
        argc--;
    }

    const char *input = argv[0];
    argv++;
    argc--;

    DataExtractor extractor;

    if (enable_streams(argc, argv, &extractor) != 0) {
        print_help();
        return 1;
    }

    run_data_extractor(input, run_state_estimators, &extractor);

    if (plot_data(argc, argv, &extractor) != 0) {
        print_help();
        return 1;
    }

    return 0;
}
