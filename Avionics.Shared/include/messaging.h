#ifndef MESSAGING_H
#define MESSAGING_H
#include "telemetry.h"
#include <stdbool.h>
#include "messaging_defs.h"
#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MESSAGING_PRODUCER(name, packet_source, packet_source_mask, heap_size) \
    TELEMETRY_ALLOCATOR(name##_allocator, heap_size) \
    static message_producer_t name = {packet_source, packet_source_mask, &name##_allocator, NULL};

#ifdef WIN32
// On windows we don't bother allocating the buffer as it won't be used
#define MESSAGING_CONSUMER(name, packet_source, packet_source_mask, message_metadata, message_metadata_mask, consumer_func, mailbox_size) \
    static message_consumer_t name = {packet_source, packet_source_mask, message_metadata, message_metadata_mask, consumer_func, mailbox_size, NULL, NULL};
#else
#define MESSAGING_CONSUMER(name, packet_source, packet_source_mask, message_metadata, message_metadata_mask, consumer_func, mailbox_size) \
    static volatile msg_t name##_mailbox_buffer[mailbox_size] MEMORY_BUFFER_ATTRIBUTES; \
    static message_consumer_t name = {packet_source, packet_source_mask, message_metadata, message_metadata_mask, consumer_func, mailbox_size, name##_mailbox_buffer, NULL};
#endif

void messaging_start(void);

bool messaging_started(void);

// Initialise a producer - returns false on error
bool messaging_producer_init(message_producer_t* producer);

// Initialise a consumer - returns false on error
bool messaging_consumer_init(message_consumer_t* consumer);

// Send a mesage from the specified producer
// A copy of the data will be made, so you can freely modify/release the data after this call
messaging_send_return_codes messaging_producer_send(message_producer_t* producer, uint16_t tag, message_metadata_t flags, const uint8_t* data, uint16_t length);

// messaging_producer_send is the reccomended method to send packets.
// This method is intended for predominantely internal use.
// NB: A copy will NOT be made of the provided packet- on calling this function
// ownership is transferred to the messaging system - you should not modify it
// or delete it after this call. It will be deleted for you when appropriate
// In the event of an error the packet will still be deleted
// The packet MUST have been allocated by the telemetry allocator component
messaging_send_return_codes messaging_send(telemetry_t* packet, message_metadata_t flags);

// Consume the next packet in the consumer's buffer
// If silent is specified will not invoke the callback function
// This function must be capable of being called recursively (e.g consumer pauses during callback)
messaging_receive_return_codes messaging_consumer_receive(message_consumer_t* consumer_id, bool blocking, bool silent);

void messaging_pause_consumer(message_consumer_t* consumer, bool flush_buffer);

void messaging_resume_consumer(message_consumer_t* consumer);

#ifdef __cplusplus
}
#endif

#endif /* MESSAGING_H */
