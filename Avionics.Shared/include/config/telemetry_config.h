#ifndef TELEMETRY_CONFIG_H
#define TELEMETRY_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

// Simple macros to make code more readable and perform validity checking
#define TELEMETRY_SOURCE_MASK(tag_length) 0b11111111111 - ((1 << tag_length)-1)
#define TELEMETRY_SOURCE(source, tag_length) (source << tag_length)
#define TELEMETRY_ID(source, tag, tag_length) (((source) << tag_length) & 0b11111111111) | (tag & ((1 << tag_length)-1))

typedef enum {
    telemetry_source_mask_component_state = TELEMETRY_SOURCE_MASK(0),
    telemetry_source_mask_ms5611 = TELEMETRY_SOURCE_MASK(0),
    telemetry_source_mask_mpu9250 = TELEMETRY_SOURCE_MASK(0),
    telemetry_source_mask_state_estimators = TELEMETRY_SOURCE_MASK(2),

} telemetry_source_mask_t;

typedef enum {
    telemetry_source_component_state = TELEMETRY_SOURCE(0b00000000000, 0),

    // Sensors
    telemetry_source_ms5611 = TELEMETRY_SOURCE(0b00000001000, 0),
    telemetry_source_mpu9250 = TELEMETRY_SOURCE(0b00000001001, 0),
    telemetry_source_state_estimators = TELEMETRY_SOURCE(0b000000011,2),

} telemetry_source_t;

typedef enum {
    telemetry_id_component_state_update = TELEMETRY_ID(0b00000000000, 0, 0),
    telemetry_id_ms5611_data = TELEMETRY_ID(0b00000001000, 0, 0),
    telemetry_id_mpu9250_data = TELEMETRY_ID(0b00000001001, 0, 0),
    telemetry_id_state_estimators_state_estimate = TELEMETRY_ID(0b000000011, 0b00, 2),
} telemetry_id_t;

typedef enum {
    telemetry_origin_imu = 0,
    telemetry_origin_avionics_gui = 1
} telemetry_origin_t;

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_CONFIG_H */
