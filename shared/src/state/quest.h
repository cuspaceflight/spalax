#ifndef QUEST_H
#define QUEST_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void quest_estimate(const float observations[2][3], const float references[2][3], const float weights[2], float q_out[4]);

#ifdef __cplusplus
}
#endif

#endif
