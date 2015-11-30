#ifndef TIME_UTILS_H
#define TIME_UTILS_H
#include <stdint.h>

#ifdef WIN32
// The base clock frequency of the chip - not sure if this a decent approximation of halGetCounterFrequency
// However, it is consistent with the data having an update rate of 1kHz
#define CLOCK_FREQUENCY 168000000.0f
#else
#define CLOCK_FREQUENCY (float)halGetCounterFrequency();
#endif


uint32_t clocks_between(uint32_t previous, uint32_t current);

// Detects overflow in the 32-bit timer and uses this to generate a 64-bit cycle count
// Provided this method is called more frequently than the 32-bit timer overflows this will work
// TODO: Investigate whether it is possible to do this better (i.e using interrupts or something)
uint64_t get_64bit_time();

void set_simulation_time(uint32_t t);

#endif  /* TIME_UTILS_H */