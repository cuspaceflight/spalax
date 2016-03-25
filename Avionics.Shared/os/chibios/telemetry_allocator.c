#include "telemetry_allocator.h"
#include "hal.h"
#include "component_state.h"

struct telemetry_allocator_impl_t {
    MemoryHeap memory_heap;
};

#define MAX_NUM_ALLOCATORS 32

// The index in the allocator_pool to use
// when registering the next allocator
static volatile int cur_allocator_pool_index = 0;

// Allocators are allocated from here
static telemetry_allocator_impl_t allocator_pool[MAX_NUM_ALLOCATORS];

// Used to make registration synchronous
// This protects cur_allocator_pool_index
// and the allocation of sections of allocator_heap_buffer
// This could be made more fine-grained but I see no major reason to
static Mutex register_mutex;

static volatile bool is_started = false;

void telemetry_allocator_start(void) {
    chMtxInit(&register_mutex);
    cur_allocator_pool_index = 0;

    memory_barrier_release();

    is_started = true;

    COMPONENT_STATE_UPDATE(avionics_component_telemetry_allocator, state_ok);
}

bool telemetry_allocator_started(void) {
    bool is_started_local = is_started;
    memory_barrier_acquire();
    return is_started_local;
}

// Creates a telemetry allocator
// This is guaranteed to have at least 'heap_size' memory available for it to use
// Returns true on success - otherwise returns false
bool telemetry_allocator_init(telemetry_allocator_t* allocator) {
    chMtxLock(&register_mutex);
    if (allocator->impl != NULL) {
        chMtxUnlock(); // register_mutex
        return true; // We assume has already been allocated
    }
    if (cur_allocator_pool_index >= MAX_NUM_ALLOCATORS) {
        chMtxUnlock(); // register_mutex
        COMPONENT_STATE_UPDATE(avionics_component_telemetry_allocator, state_error);
        return false; // TODO: Log some sort of error
    }

    allocator->impl = &(allocator_pool[cur_allocator_pool_index]);
    cur_allocator_pool_index++;

    chHeapInit(&allocator->impl->memory_heap, (void*)allocator->heap_buffer, allocator->heap_size);

    chMtxUnlock(); // register_mutex
    return true;
}

// Allocates a packet with the provided data
// NB will not initialize any fields
// the payload_size should be in bytes
// Will return NULL if allocation fails for any reason
telemetry_t* telemetry_allocator_alloc(telemetry_allocator_t* allocator, uint32_t payload_size) {
    // TODO: Enforce some sort of max payload_size limit
    if (allocator->impl == NULL) {
        COMPONENT_STATE_UPDATE(avionics_component_telemetry_allocator, state_error);
        return NULL;
    }
    telemetry_t* ret = chHeapAlloc(&allocator->impl->memory_heap, sizeof(telemetry_t));
    if (ret == NULL) {
        COMPONENT_STATE_UPDATE(avionics_component_telemetry_allocator, state_error);
        return NULL;
    }
    uint8_t* payload = chHeapAlloc(&allocator->impl->memory_heap, payload_size);
    if (payload == NULL) {
        chHeapFree(ret);
        COMPONENT_STATE_UPDATE(avionics_component_telemetry_allocator, state_error);
        return NULL;
    }
    ret->payload = payload;
    return ret;
}

// Frees an allocated packet
void telemetry_allocator_free(telemetry_t* packet) {
    if (packet == NULL)
         return;
    // TODO: Verify that chibios chHeapFree is happy to be called with NULL pointers
    chHeapFree(packet->payload);
    chHeapFree(packet);
}
