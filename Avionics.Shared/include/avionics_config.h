#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

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

typedef struct avionics_config_t {
    telemetry_origin_t origin;
} avionics_config_t;

// We use the messaging system's metadata to store flags for interface drivers
typedef enum {
    message_flags_dont_send_over_can = 1 << 0,
    message_flags_dont_send_over_usb = 1 << 1,
    message_flags_dont_send_over_rs232 = 1 << 2,
    message_flags_dont_send_over_radio = 1 << 3,
} message_metadata_flags_t;

// This should be defined somewhere with the local configuration e.g in main.c
extern const avionics_config_t local_config;

#ifdef __cplusplus
}
#endif

#endif
