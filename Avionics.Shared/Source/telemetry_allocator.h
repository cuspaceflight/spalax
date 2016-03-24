#ifndef TELEMETRY_ALLOCATOR_H
#define TELEMETRY_ALLOCATOR_H
#include <stdbool.h>
#include <stdint.h>
#include "telemetry.h"
#include "platform.h"

// Implementation specific type
typedef struct telemetry_allocator_impl_t telemetry_allocator_impl_t;

typedef struct telemetry_allocator_t {
    const uint32_t heap_size;
    volatile char* const heap_buffer;
    telemetry_allocator_impl_t* impl; // Used for storing implementation specific data
} telemetry_allocator_t;


#ifdef WIN32
// On windows we don't bother allocating the buffer as it won't be used
#define TELEMETRY_ALLOCATOR(name, heap_size) \
    static telemetry_allocator_t name = {heap_size, NULL, NULL};
#else
#define TELEMETRY_ALLOCATOR(name, heap_size) \
    static volatile char name##_buffer[heap_size] MEMORY_BUFFER_ATTRIBUTES; \
    static telemetry_allocator_t name = {heap_size, name##_buffer, NULL};
#endif

// Initialises the telemetry allocation component
void init_telemetry_allocators(void);

// Creates a telemetry allocator
// This is guaranteed to have at least 'heap_size' memory available for it to use
// Returns true on success - otherwise returns false
bool telemetry_allocator_init(telemetry_allocator_t* allocator);

// Allocates a packet with the provided data
// NB will not initialize any fields
// the payload_size should be in bytes
// Will return NULL if allocation fails for any reason
telemetry_t* telemetry_allocator_alloc(telemetry_allocator_t* allocator, uint32_t payload_size);

// Frees an allocated packet
void telemetry_allocator_free(telemetry_t* packet);

#endif /* TELEMETRY_ALLOCATOR_H */
