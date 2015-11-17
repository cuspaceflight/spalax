/*
 * ADXL3x5 Driver (ADXL345, ADXL375)
 * M2FC
 * 2014 Adam Greig, Cambridge University Spaceflight
 */

#ifndef ADIS16405_H
#define ADIS16405_H

#include "ch.h"
#include "hal.h"

msg_t adis16405_thread(void *arg);

/* Interrupt callbacks for EXTI. */
void adis16405_wakeup(EXTDriver *extp, expchannel_t channel);

#endif /* ADIS16405_H */
