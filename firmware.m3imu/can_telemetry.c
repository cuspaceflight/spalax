#include "can_telemetry.h"
#include "component_state.h"
#include "m3can.h"
#include "messaging.h"
#include "avionics_config.h"
#include <string.h>

static bool transmit_packet(const telemetry_t* packet, message_metadata_t metadata);

MESSAGING_CONSUMER(can_telemetry_messaging_consumer, 0, 0, 0, message_flags_dont_send_over_can, transmit_packet, 20);

TELEMETRY_ALLOCATOR(can_telemetry_allocator, 1024);

void can_telemetry_start(void) {
    messaging_consumer_init(&can_telemetry_messaging_consumer);
    telemetry_allocator_init(&can_telemetry_allocator);


    COMPONENT_STATE_UPDATE(avionics_component_can_telemetry, state_ok);
}

static bool transmit_packet(const telemetry_t* packet, message_metadata_t metadata) {
    (void)metadata;
    if (packet->header.origin == local_config.origin) {
        if (packet->header.length <= 8) {
            // TODO: RTR?
            can_send((packet->header.id << 5) | CAN_ID_M3IMU, FALSE, packet->payload, packet->header.length);
        }
    }
    return true;
}

void can_recv(uint16_t msg_id, bool can_rtr, uint8_t *data, uint8_t datalen) {
    (void)can_rtr;
    telemetry_t* packet = telemetry_allocator_alloc(&can_telemetry_allocator, datalen);
    if (packet == NULL)
        return;
    memcpy(data, packet->payload, datalen);
    packet->header.id = (msg_id >> 5) & 0x3F;
    packet->header.length = datalen;
    packet->header.origin = msg_id & 0x1F;
    messaging_send(packet, 0);
}

void can_telemetry_transmit_thread(void* arg) {
    (void)arg;
    while (true) {
        messaging_consumer_receive(&can_telemetry_messaging_consumer, true, false);
        chThdYield();
    }
}
