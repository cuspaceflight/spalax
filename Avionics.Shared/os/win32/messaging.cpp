#include "messaging.h"
#include <string.h>
#include "platform.h"
#include <TQueue.h>
#include <atomic>
#include "component_state.h"
#include "avionics_config.h"

#define MAX_NUM_CONSUMERS 20
#define MAX_NUM_PRODUCERS 20

#define MAX_NUM_TELEMETRY_REFS 1000

struct TelemetryRef {
    telemetry_t* packet;
    message_metadata_t flags;

    TelemetryRef(telemetry_t* packet, message_metadata_t flags) : packet(packet), flags(flags) {
    }

    ~TelemetryRef() {
        telemetry_allocator_free(packet);
    }
};

// This may be used for load tracking later on so is included
struct message_producer_impl_t {
    message_producer_t* parent;
};

struct message_consumer_impl_t {
    TQueue<std::shared_ptr<TelemetryRef>> mailbox;
    message_consumer_t* parent;
    volatile bool is_paused;
};

static std::atomic<uint32_t> cur_consumer_pool_index = 0;

static message_consumer_impl_t consumer_pool[MAX_NUM_CONSUMERS];

static std::mutex consumer_register_mutex;

static std::atomic<uint32_t> cur_producer_pool_index = 0;

static message_producer_impl_t producer_pool[MAX_NUM_PRODUCERS];

static std::mutex producer_register_mutex;

static std::atomic<bool> is_started = false;

// Defined in component_state.c
extern "C" void component_state_register_with_messaging(void);

void messaging_start(void) {
    is_started.store(true, std::memory_order_release);
    component_state_register_with_messaging();
    COMPONENT_STATE_UPDATE(avionics_component_messaging, state_ok);
}

bool messaging_started(void) {
    return is_started.load(std::memory_order_acquire);
}

// Initialise a producer - returns false on error
extern "C" bool messaging_producer_init(message_producer_t* producer) {
    std::lock_guard<std::mutex> lock(producer_register_mutex);
    if (producer->impl != nullptr)
        return true; // We assume it has already been initialised

    if (cur_producer_pool_index >= MAX_NUM_PRODUCERS) {
        COMPONENT_STATE_UPDATE(avionics_component_messaging, state_error);
        return false;
    }
    producer->impl = &producer_pool[cur_producer_pool_index];

    // Perform any initialisation
    producer->impl->parent = producer;
    telemetry_allocator_init(producer->telemetry_allocator);

    cur_producer_pool_index.fetch_add(1, std::memory_order_release);
    return true;
}

// Initialise a consumer - returns false on error
extern "C" bool messaging_consumer_init(message_consumer_t* consumer) {
    std::lock_guard<std::mutex> lock(consumer_register_mutex);
    if (consumer->impl != nullptr)
        return true; // We assume it has already been initialised

    if (cur_consumer_pool_index >= MAX_NUM_CONSUMERS) {
        COMPONENT_STATE_UPDATE(avionics_component_messaging, state_error);
        return false;
    }
    consumer->impl = &consumer_pool[cur_consumer_pool_index];
    consumer->impl->parent = consumer;

    // Perform any initialisation

    cur_consumer_pool_index.fetch_add(1, std::memory_order_release);
    return true;
}


static bool messaging_consumer_enqueue_packet(message_consumer_t* consumer, const std::shared_ptr<TelemetryRef>& ref) {
    consumer->impl->mailbox.enqueue(ref);
    return true;
}

messaging_send_return_codes messaging_send(telemetry_t* packet, message_metadata_t flags) {
    auto ref = std::make_shared<TelemetryRef>(packet, flags);
    bool enqueue_successful = true;

    // We create a local copy as it frees up the compiler
    // If a consumer registers during this call it isn't a massive deal that
    // we won't pass it the packet.
    uint32_t num_consumers = cur_consumer_pool_index.load(std::memory_order_acquire);

    for (uint32_t i = 0; i < num_consumers; ++i) {
        message_consumer_t* consumer = consumer_pool[i].parent;
        if (!consumer->impl->is_paused
            && (consumer->packet_source_mask & packet->header.id) == consumer->packet_source
            && (consumer->message_metadata_mask & flags) == consumer->message_metadata
            && !messaging_consumer_enqueue_packet(consumer, ref)) {
            enqueue_successful = false;
        }
    }

    return enqueue_successful ? messaging_send_ok : messaging_send_consumer_buffer_full;
}

// Send a mesage from the specified producer
// A copy of the data will be made, so you can freely modify/release the data after this call
extern "C" messaging_send_return_codes messaging_producer_send(message_producer_t* producer, message_metadata_t flags, const uint8_t* data, uint16_t length) {
    if (producer->impl == NULL) {
        COMPONENT_STATE_UPDATE(avionics_component_messaging, state_error);
        return messaging_send_invalid_producer;
    }

    telemetry_t* packet = telemetry_allocator_alloc(producer->telemetry_allocator, length);
    if (packet == nullptr) {
        COMPONENT_STATE_UPDATE(avionics_component_messaging, state_error);
        return messaging_send_producer_heap_full;
    }

    memcpy(packet->payload, data, length);

    // We have already checked the tag and source don't overlap earlier
    packet->header.id = producer->packet_id;
    packet->header.length = length;
    packet->header.timestamp = platform_get_counter_value();
    packet->header.origin = local_config.origin;

    return messaging_send(packet, flags);
}

// Consume the next packet in the consumer's buffer
// If silent is specified will not invoke the callback function
extern "C" messaging_receive_return_codes messaging_consumer_receive(message_consumer_t* consumer, bool blocking, bool silent) {
    if (consumer->impl == nullptr) {
        COMPONENT_STATE_UPDATE(avionics_component_messaging, state_error);
        return messaging_receive_invalid_consumer;
    }

    if (!blocking && consumer->impl->mailbox.isEmpty())
        return messaging_receive_buffer_empty;

    auto ref = consumer->impl->mailbox.dequeue();
    if (!silent) {
        if (!consumer->consumer_func(ref->packet, ref->flags)) {
            return messaging_receive_callback_error;
        }
    }

    return messaging_receive_ok;
}

void messaging_pause_consumer(message_consumer_t* consumer, bool flush_buffer) {
    consumer->impl->is_paused = true;
    if (flush_buffer)
        while (messaging_consumer_receive(consumer, false, true) != messaging_receive_buffer_empty);
}

void messaging_resume_consumer(message_consumer_t* consumer) {
    consumer->impl->is_paused = false;
}
