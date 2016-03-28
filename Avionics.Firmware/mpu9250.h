#ifndef MPU9250_H
#define MPU9250_H

#include "ch.h"
#include "hal.h"
#include "compilermacros.h"
#include "mpu9250_config.h"

msg_t mpu9250_thread(void *arg);

/* Interrupt callbacks for EXTI. */
void mpu9250_wakeup(EXTDriver *extp, expchannel_t channel);

#endif /* MPU9250_H */
