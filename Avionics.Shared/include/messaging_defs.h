#ifndef MESSAGING_DEFS_H
#define MESSAGING_DEFS_H
#include "telemetry_allocator.h"
#include "compilermacros.h"
#include "messaging_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint16_t message_metadata_t;
typedef struct message_producer_impl_t message_producer_impl_t;
typedef struct message_consumer_impl_t message_consumer_impl_t;

typedef struct message_producer_t {
    const uint16_t packet_source;
    const uint16_t packet_source_mask;
    telemetry_allocator_t* const telemetry_allocator;
    message_producer_impl_t* impl; // Used for implementation specific data
} message_producer_t;

typedef struct message_consumer_t {
    const uint16_t packet_source;
    const uint16_t packet_source_mask;
    const message_metadata_t message_metadata;
    const message_metadata_t message_metadata_mask;
    bool(* const consumer_func)(telemetry_t*, message_metadata_t);
    const uint32_t mailbox_size; // Maximum number of packets which can be waiting for processing
    volatile int32_t* const mailbox_buffer;
    message_consumer_impl_t* impl; // Used for implementation specific data
} message_consumer_t;

typedef enum {
    messaging_send_ok, // Packet sent sucessfully
    messaging_send_producer_heap_full, // Insufficient space in producer heap to allocate packet
    messaging_send_invalid_producer, // The producer is invalid
    messaging_send_invalid_tag, // The tag doesn't match the provided source mask
    messaging_send_internal_pool_full, // The internal memory pool of the component is full
    messaging_send_consumer_buffer_full // At least one consumer dropped the packet
} messaging_send_return_codes;

typedef enum {
    messaging_receive_ok, // Successfully processed a packet
    messaging_receive_buffer_empty, // No packets in buffer to process
    messaging_receive_invalid_consumer, // The consumer is invalid
    messaging_receive_callback_error, // consumer_func returned false
} messaging_receive_return_codes;


#ifdef __cplusplus
}
#endif

#endif /* MESSAGING_DEFS_H */
