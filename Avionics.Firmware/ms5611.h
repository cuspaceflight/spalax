/*
 * MS5611-01BA03 Driver
 * M2FC
 * 2014 Adam Greig, Cambridge University Spaceflight
 */

#ifndef MS5611_H
#define MS5611_H

#include <ch.h>

/* The main thread. Run this. */
msg_t ms5611_thread(void *arg);

#endif /* MS5611_H */
