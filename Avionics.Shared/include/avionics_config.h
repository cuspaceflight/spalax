#ifndef AVIONICS_CONFIG_H
#define AVIONICS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

//
// Telemetry ID Assignments
//

// Simple macros to make code more readable and perform validity checking
#define TELEMETRY_SOURCE_MASK(tag_length) 0b11111111111 - ((1 << tag_length)-1)
#define TELEMETRY_SOURCE(source, tag_length) (source << tag_length)
#define TELEMETRY_ID(source, tag, tag_length) (((source) << tag_length) & 0b11111111111) | (tag & ((1 << tag_length)-1))

typedef enum {
    telemetry_source_mask_component_state = TELEMETRY_SOURCE_MASK(0),
    telemetry_source_mask_state_estimators = TELEMETRY_SOURCE_MASK(2),

} telemetry_source_mask_t;

typedef enum {
    telemetry_source_component_state = TELEMETRY_SOURCE(0b00000000000, 0),
    telemetry_source_state_estimators = TELEMETRY_SOURCE(0b100000,2),

} telemetry_source_t;

typedef enum {
    telemetry_id_component_state_update = TELEMETRY_ID(0b00000000000, 0, 0),
    telemetry_id_state_estimators_quaternion = TELEMETRY_ID(0b100000, 0b00, 2),
} telemetry_id_t;

typedef enum {
    telemetry_origin_imu = 0,
    telemetry_origin_avionics_gui = 1
} telemetry_origin_t;

//
// Messaging System Config
//

// We use the messaging system's metadata to store flags for interface drivers
typedef enum {
    message_flags_dont_send_over_can = 1 << 0,
    message_flags_dont_send_over_usb = 1 << 1,
    message_flags_dont_send_over_rs232 = 1 << 2,
    message_flags_dont_send_over_radio = 1 << 3,
} message_metadata_flags_t;

//
// Component State Tracking
//


typedef enum {
    avionics_component_messaging = 0,
    avionics_component_telemetry_allocator,
    avionics_component_max // This must be last
} avionics_component_t;

//
// Local System Config
//

typedef void(*avionics_error_handler_t)(avionics_component_t component, int line);

typedef struct avionics_config_t {
    // The origin of the local system
    const telemetry_origin_t origin;

    // NB: This will be called from a variety of threads
    // and so should be a thread safe function
    const avionics_error_handler_t error_handler;
} avionics_config_t;

// This should be defined somewhere with the local configuration e.g in main.c
extern const avionics_config_t local_config;

#ifdef __cplusplus
}
#endif

#endif /* AVIONICS_CONFIG_H */
