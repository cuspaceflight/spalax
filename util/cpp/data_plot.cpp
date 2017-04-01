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

int main(int argc, char *argv[]) {
    if (argc < 4 || argc % 2 != 0) {
        std::cerr << "Invalid arguments - expected <Input> [<Output> <Format>]...";
        return 1;
    }
    const char *input = argv[1];


    setBoardConfig(BoardConfigSpalaxBrokenSD);

    component_state_start(update_handler, false);
    messaging_all_start_options(false, false);

    DataExtractor extractor;

    enable_streams(argc, argv, &extractor);

    run_data_extractor(input, &extractor);

    plot_data(argc, argv, &extractor);
    return 0;
}
