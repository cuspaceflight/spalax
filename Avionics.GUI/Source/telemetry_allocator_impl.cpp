extern "C" {
    #include "telemetry_allocator.h"
}
#include <new>


// TODO: Enforce heap limits

void init_telemetry_allocators(void) {
    
}

extern "C" bool telemetry_allocator_init(telemetry_allocator_t* allocator) {
    // We don't need to do anything
    return true;
}

extern "C" telemetry_t* telemetry_allocator_alloc(telemetry_allocator_t* allocator, uint32_t payload_size) {
    // TODO: Enforce some sort of max payload_size limit

    // We use the no throw versions to prevent C++ exceptions crossing language boundaries
    auto ret = new (std::nothrow) telemetry_t();
    if (ret == nullptr)
        return nullptr;
    auto data = new (std::nothrow) uint8_t[payload_size];
    if (data == nullptr) {
        delete ret;
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