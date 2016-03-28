#include "time_utils.h"
#include "logging.h"

uint32_t clocks_between(uint32_t previous, uint32_t current) {
    uint64_t current_larger = (uint64_t)current;
    if (previous > current)
        current_larger += 0xFFFFFFFF; // We have overflowed
    current_larger -= previous;
    return (uint32_t)current_larger;
}


uint32_t last_time_32 = 0;
uint64_t timestep_correction = 0;

uint64_t get_64bit_time() {
    // TODO: Make this thead-safe
    uint32_t time_32 = platform_get_counter_value();
    if (time_32 < last_time_32) {
        PRINT("uint32_t clock rollover\n");
        timestep_correction += 0xFFFFFFFF;
    }
    last_time_32 = time_32;
    return timestep_correction + (uint64_t)time_32;
}

