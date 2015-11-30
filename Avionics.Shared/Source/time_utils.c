#include "time_utils.h"
#include "logging.h"

uint32_t clocks_between(uint32_t previous, uint32_t current) {
    uint64_t current_larger = (uint64_t)current;
    if (previous > current)
        current_larger += 0xFFFFFFFF; // We have overflowed
    current_larger -= previous;
    return (uint32_t)current_larger;
}

#ifdef WIN32

uint32_t simulation_time = 0;

void set_simulation_time(uint32_t t) {
    simulation_time = t;
}

uint32_t get_32_bit_time() {
    return simulation_time;
}

#else

uint32_t get_32_bit_time() {
    return halGetCounterValue();
}

#endif

uint32_t last_time_32 = 0;
uint64_t timestep_correction = 0;

uint64_t get_64bit_time() {
    uint32_t time_32 = get_32_bit_time();
    if (time_32 < last_time_32) {
        PRINT("uint32_t clock rollover\n");
        timestep_correction += 0xFFFFFFFF;
    }
    last_time_32 = time_32;
    return timestep_correction + (uint64_t)time_32;
}

