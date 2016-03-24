#ifndef TELEMETRY_DEFS_H
#define TELEMETRY_DEFS_H

// Simple macros to make code more readable and perform validity checking
#define TELEMETRY_SOURCE_MASK(tag_length) 0b11111111111 - ((1 << tag_length)-1)
#define TELEMETRY_SOURCE(source, tag_length) (source << tag_length)
#define TELEMETRY_ID(source, tag, tag_length) (((source) << tag_length) & 0b11111111111) | (tag & ((1 << tag_length)-1))


typedef enum {
    telemetry_source_mask_state_estimators = TELEMETRY_SOURCE_MASK(2),
} telemetry_source_mask_t;

typedef enum {
    telemetry_source_state_estimators = TELEMETRY_SOURCE(0b000000,2),
} telemetry_source_t;

typedef enum {
    telemetry_id_state_estimators_quaternion = TELEMETRY_ID(0b000000, 0b11, 2),
} telemetry_id_t;


typedef enum {
    telemetry_origin_imu = 0,
    telemetry_origin_avionics_gui = 1
} telemetry_origin_t;

#ifdef WIN32
#define TELEMETRY_ORIGIN telemetry_origin_avionics_gui
#else
#define TELEMETRY_ORIGIN telemetry_origin_imu
#endif

#endif /* TELEMETRY_DEFS_H */
