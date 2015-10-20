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


static uint32_t clocks_between(uint32_t previous, uint32_t current) {
	uint64_t current_larger = (uint64_t)current;
	if (previous > current)
		current_larger += 0xFFFFFFFF; // We have overflowed
	current_larger -= previous;
	return (uint32_t)current_larger;
}




#endif  /* TIME_UTILS_H */