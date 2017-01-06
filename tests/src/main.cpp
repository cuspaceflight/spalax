#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "component_state.h"
#include "avionics_config.h"


static void update_handler(avionics_component_t component, avionics_component_state_t state, int line) {
    if (state == state_initializing)
        printf("Component: %i State Update: Initializing\n", component);
    else if (state == state_ok)
        printf("Component: %i State Update: OK\n", component);
    else if (state == state_error)
        printf("Component: %i State Error: OK\n", component);
}

avionics_config_t local_config = { update_handler};

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
