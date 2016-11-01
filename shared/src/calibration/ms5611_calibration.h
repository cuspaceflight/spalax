#ifndef MS5611_CONFIG_H
#define MS5611_CONFIG_H
#include "compilermacros.h"
#include <stdint.h>
#include "telemetry_packets.h"

#ifdef __cplusplus
extern "C" {
#endif

float ms5611_get_altitude(float pressure);


#ifdef __cplusplus
}
#endif

#endif /* MS5611_CONFIG_H */
