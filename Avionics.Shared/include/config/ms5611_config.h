#ifndef MS5611_CONFIG_H
#define MS5611_CONFIG_H
#include "compilermacros.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ms5611data_t {
    int32_t temperature; // Temperature in Celcius
    int32_t pressure; // Pressure in Pascals
} ms5611data_t;

STATIC_ASSERT(sizeof(ms5611data_t) == 8, ms5611data_padded);

#ifdef __cplusplus
}
#endif

#endif /* MS5611_CONFIG_H */
