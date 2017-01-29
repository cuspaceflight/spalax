#ifndef QUEST_H
#define QUEST_H
#include <stdint.h>
#include "compilermacros.h"

#ifdef __cplusplus
extern "C" {
#endif

int quest_estimate(const float observations[2][3], const float references[2][3], const float weights[2], float q_out[4]);

//#define BUILD_Q_METHOD
#ifdef BUILD_Q_METHOD
int davenport_q_method(const float **observations, const float **references, const float *weights, float *q_out);
#endif

#ifdef __cplusplus
}
#endif

#endif
