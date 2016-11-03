#ifndef QUEST_H
#define QUEST_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void quest_estimate(const float accel[3], const float mag[3], float q_out[4]);

#ifdef __cplusplus
}
#endif

#endif
