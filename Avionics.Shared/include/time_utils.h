#ifndef TIME_UTILS_H
#define TIME_UTILS_H
#include <stdint.h>
#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CLOCK_FREQUENCY platform_get_counter_frequency()

uint32_t clocks_between(uint32_t previous, uint32_t current);

// Detects overflow in the 32-bit timer and uses this to generate a 64-bit cycle count
// Provided this method is called more frequently than the 32-bit timer overflows this will work
// TODO: Investigate whether it is possible to do this better (i.e using interrupts or something)
uint64_t get_64bit_time();

#ifdef __cplusplus
}
#endif

#endif  /* TIME_UTILS_H */
