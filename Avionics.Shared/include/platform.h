#ifndef PLATFORM_H
#define PLATFORM_H
#include <stdint.h>

#ifdef WIN32
#define MEMORY_BUFFER_ATTRIBUTES
#else
#include "ch.h"
#define MEMORY_BUFFER_ATTRIBUTES __attribute__((aligned(sizeof(stkalign_t)))) __attribute__((section(".ccm")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

void memory_barrier_acquire(void);

void memory_barrier_release(void);

// This file contains declarations which are to be defined elsewhere

uint32_t platform_get_counter_value(void);

void platform_set_counter_value(uint32_t value);

uint32_t platform_get_counter_frequency(void);

#ifdef __cplusplus
}
#endif

#endif /*PLATFORM_H*/
