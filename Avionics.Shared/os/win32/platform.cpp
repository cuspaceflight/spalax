#include "platform.h"

uint32_t counter_value = 0;

extern "C" void platform_set_counter_value(uint32_t t) {
    counter_value = t;
}

extern "C" uint32_t platform_get_counter_value() {
    return counter_value;
}

extern "C" uint32_t platform_get_counter_frequency() {
    // The base clock frequency of the chip - not sure if this a decent approximation of halGetCounterFrequency
    // However, it is consistent with the data having an update rate of 1kHz
    return 168000000;
}