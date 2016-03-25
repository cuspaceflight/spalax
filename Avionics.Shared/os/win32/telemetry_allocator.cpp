#include "telemetry_allocator.h"
#include <new>
#include "component_state.h"

// TODO: Enforce heap limits

void init_telemetry_allocators(void) {
    COMPONENT_STATE_UPDATE(avionics_component_telemetry_allocator, state_ok);
}

extern "C" bool telemetry_allocator_init(telemetry_allocator_t* allocator) {
    // We don't need to do anything
    return true;
}

extern "C" telemetry_t* telemetry_allocator_alloc(telemetry_allocator_t* allocator, uint32_t payload_size) {
    // TODO: Enforce some sort of max payload_size limit

    // We use the no throw versions to prevent C++ exceptions crossing language boundaries
    // Although if new is failing something is seriously wrong...
    auto ret = new (std::nothrow) telemetry_t();
    if (ret == nullptr) {
        COMPONENT_STATE_UPDATE(avionics_component_telemetry_allocator, state_error);
        return nullptr;
    }
    auto data = new (std::nothrow) uint8_t[payload_size];
    if (data == nullptr) {
        delete ret;
        COMPONENT_STATE_UPDATE(avionics_component_telemetry_allocator, state_error);
        return nullptr;
    }
    ret->payload = data;
    return ret;
}

extern "C" void telemetry_allocator_free(telemetry_t* packet) {
    if (packet == nullptr)
        return;
    delete packet->payload;
    delete packet;
}
