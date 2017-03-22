#ifndef STATE_ESTIMATE_H
#define STATE_ESTIMATE_H
#include <stdint.h>
#include "compilermacros.h"

#ifdef __cplusplus
extern "C" {
#endif

void state_estimate_init();

void state_estimate_thread(void *arg);


#ifdef MESSAGING_OS_STD
void state_estimate_terminate();
#endif

#ifdef __cplusplus
}
#endif

#endif
