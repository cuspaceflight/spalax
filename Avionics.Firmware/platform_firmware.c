#include "platform.h"
#include "hal.h"

uint32_t platform_get_counter_value() {
    return halGetCounterValue();
}

uint32_t platform_get_counter_frequency() {
    return halGetCounterFrequency();
}
