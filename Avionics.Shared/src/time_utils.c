#include "time_utils.h"
#include "logging.h"

uint32_t clocks_between(uint32_t previous, uint32_t current) {
    uint64_t current_larger = (uint64_t)current;
    if (previous > current)
        current_larger += 0xFFFFFFFF; // We have overflowed
    current_larger -= previous;
    return (uint32_t)current_larger;
}
