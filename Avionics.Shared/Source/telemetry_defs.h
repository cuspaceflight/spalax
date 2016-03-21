#ifndef TELEMETRY_DEFS_H
#define TELEMETRY_DEFS_H

typedef enum {
    telemetry_mode_string = 0,
    telemetry_mode_int64 = 1,
    telemetry_mode_uint64 = 2,
    telemetry_mode_int32 = 3,
    telemetry_mode_uint32 = 4,
    telemetry_mode_int16 = 5,
    telemetry_mode_uint16 = 6,
    telemetry_mode_int8 = 7,
    telemetry_mode_uint8 = 8,
    telemetry_mode_float = 9,
    telemetry_mode_double = 10,
} telemetry_mode_t;

typedef enum {
    telemetry_source_system = 0x0000,
    telemetry_source_calibration = 0x0001,
    telemetry_source_imu = 0x0002,
    telemetry_source_state_estimation = 0x0005,
    telemetry_source_wildcard = 0xFFFF,
} telemetry_source_t;

#endif /* TELEMETRY_DEFS_H */
