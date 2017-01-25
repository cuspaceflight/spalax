#ifndef MS5611_CONFIG_H
#define MS5611_CONFIG_H
#include "compilermacros.h"
#include <stdint.h>
#include "telemetry_packets.h"

#ifdef __cplusplus
extern "C" {
#endif

float ms5611_get_altitude(const ms5611data_t* data);


#ifdef __cplusplus
}
#endif

#endif /* MS5611_CONFIG_H */
