#ifndef KALMAN_H
#define KALMAN_H
#include <stdint.h>
#include "compilermacros.h"

#ifdef __cplusplus
extern "C" {
#endif

void kalman_init(float accel_reference[3], float magno_reference[3]);

#ifdef __cplusplus
}
#endif

#endif
