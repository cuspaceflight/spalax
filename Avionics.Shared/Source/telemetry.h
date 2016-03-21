#ifndef TELEMETRY_H
#define TELEMETRY_H
#include <stdint.h>
#include "telemetry_defs.h"

typedef struct telemetry_t {
    union {
        char string_data[16];
        int64_t int64_data[2];
        uint64_t uint64_data[2];
        int32_t int32_data[4];
        uint32_t uint32_data[4];
        int16_t int16_data[8];
        uint16_t uint16_data[8];
        int8_t int8_data[16];
        uint8_t uint8_data[16];
        float float_data[4];
        double double_data[2];
    };
    uint64_t timestamp;
    uint16_t origin; // The device that produced this packet
    uint16_t source; // The software component that produced this packet
    uint8_t tag; // The type of packet for the given source
    uint8_t mode; // The type of data
    uint16_t check_sum;
} telemetry_t;


#define PACKET_ACCEL_RAW 0x20
#define PACKET_MAG_RAW 0x23
#define PACKET_GYRO_RAW 0x24
#define PACKET_PRESSURE_RAW 0x22

void telemetry_print_data(const telemetry_t* data);

#endif /* TELEMETRY_H */
