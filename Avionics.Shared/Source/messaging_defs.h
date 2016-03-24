#ifndef MESSAGING_DEFS_H
#define MESSAGING_DEFS_H
#include "telemetry_allocator.h"

typedef uint32_t messaging_consumer_id;
typedef uint32_t messaging_producer_id;
typedef void(*messaging_consumer_func)(telemetry_t*);

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
    const messaging_consumer_func consumer_func;
    const uint32_t mailbox_size; // Maximum number of packets which can be waiting for processing
    volatile int32_t* const mailbox_buffer;
    message_consumer_impl_t* impl; // Used for implementation specific data
} message_consumer_t;

#define MESSAGING_PRODUCER(name, packet_source, packet_source_mask, heap_size) \
    TELEMETRY_ALLOCATOR(name##_allocator, heap_size) \
    static message_producer_t name = {packet_source, packet_source_mask, &name##_allocator, NULL};

#ifdef WIN32
// On windows we don't bother allocating the buffer as it won't be used
#define MESSAGING_CONSUMER(name, packet_source, packet_source_mask, consumer_func, mailbox_size) \
    static message_consumer_t name = {packet_source, packet_source_mask, consumer_func, mailbox_size, NULL, NULL};
#else
#define MESSAGING_CONSUMER(name, packet_source, packet_source_mask, consumer_func, mailbox_size) \
    static volatile msg_t name##_mailbox_buffer[mailbox_size] MEMORY_BUFFER_ATTRIBUTES; \
    static message_consumer_t name = {packet_source, packet_source_mask, consumer_func, mailbox_size, name##_mailbox_buffer, NULL};
#endif

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
} messaging_receive_return_codes;

#endif /* MESSAGING_DEFS_H */
