/*
 * MS5611-01BA03 Driver
 * M2FC
 * 2014 Adam Greig, Cambridge University Spaceflight
 */

#ifndef MS5611_H
#define MS5611_H

#include <ch.h>

typedef struct {
    uint16_t c1, c2, c3, c4, c5, c6;
} MS5611CalData;

/* The main thread. Run this. */
msg_t ms5611_thread(void *arg);

#endif /* MS5611_H */
