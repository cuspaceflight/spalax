#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H
#include <stdint.h>
#include "telemetry_allocator.h"

#ifdef __cplusplus
extern "C" {
#endif

// A generic driver for stream based interfaces, for example, rs232

typedef struct serial_interface_t {
    // Should block until input is available
    uint8_t(* const stream_get)(void);

    // Should return false if the write times out
    bool(* const stream_put)(uint8_t byte);

    // This is to allow drivers to use write buffering
    // Can be left null
    bool(* const stream_flush_write)(void);

    struct telemetry_allocator_t* const telemetry_allocator;
} serial_interface_t;

#define SERIAL_INTERFACE(name, stream_get, stream_put, stream_flush_write, heap_size) \
    TELEMETRY_ALLOCATOR(name##_allocator, heap_size) \
    static serial_interface_t name = {stream_get, stream_put, stream_flush_write, &name##_allocator};

// Will block until a valid packet is received
// The caller is the new owner of the returned packet and is
// responsible for ensuring it is cleaned up correctly
// All packets are allocated using the provided telemetry_allocator
telemetry_t* serial_interface_next_packet(serial_interface_t* serial_interface);

// Will send the packet over the provided interface
// Will return false if there is an error
bool serial_interface_send_packet(serial_interface_t* serial_interface, telemetry_t* packet);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_INTERFACE_H */
