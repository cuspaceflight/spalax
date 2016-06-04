#ifndef TIME_UTILS_H
#define TIME_UTILS_H
#include <stdint.h>
#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CLOCK_FREQUENCY platform_get_counter_frequency()

uint32_t clocks_between(uint32_t previous, uint32_t current);

#ifdef __cplusplus
}
#endif

#endif  /* TIME_UTILS_H */
