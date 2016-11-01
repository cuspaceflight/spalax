#ifndef STATE_ESTIMATE_H
#define STATE_ESTIMATE_H
#include <stdint.h>
#include "telemetry_packets.h"

#ifdef __cplusplus
extern "C" {
#endif

	void state_estimate_thread(void* arg);

#ifdef __cplusplus
}
#endif

#endif /* STATE_ESTIMATE_H */
