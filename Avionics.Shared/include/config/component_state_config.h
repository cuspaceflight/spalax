#ifndef COMPONENT_STATE_CONFIG_H
#define COMPONENT_STATE_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    avionics_component_messaging = 0,
    avionics_component_telemetry_allocator,
    avionics_component_adis16405,
    avionics_component_mpu9250,
    avionics_component_ms5611,
    avionics_component_config,
    avionics_component_max // This must be last
} avionics_component_t;

#ifdef __cplusplus
}
#endif

#endif /* COMPONENT_STATE_CONFIG_H */
