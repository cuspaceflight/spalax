#ifndef TELEMETRY_H
#define TELEMETRY_H
#include <stdint.h>
#include "telemetry_defs.h"
#include "compilermacros.h"

typedef struct telemetry_header_t {
    struct {
        unsigned int id : 11;
        unsigned int length : 12;
        unsigned int reserved : 1;
        telemetry_origin_t origin : 8;
    };
    uint32_t timestamp;
} telemetry_header_t;

typedef struct telemetry_t {
    struct telemetry_header_t header;
    uint8_t* payload;
} telemetry_t;

// Make sure compiler isn't inserting padding
STATIC_ASSERT(sizeof(telemetry_header_t) == 8, telemetry_header_padded_by_compiler);

#endif /* TELEMETRY_H */
