#ifndef QUEST_H
#define QUEST_H
#include <stdint.h>
#include "compilermacros.h"

#ifdef __cplusplus
extern "C" {
#endif

int quest_estimate(const float observations[2][3], const float references[2][3], const float weights[2], float q_out[4]);

int davenport_q_method(const float observations[][3], const float references[][3], const float a[], int n, float q_out[4]);

#ifdef __cplusplus
}
#endif

#endif
