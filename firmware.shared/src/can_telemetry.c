#include "can_telemetry.h"
#include "component_state.h"

typedef struct can_packet_t {
    uint32_t data;
} can_packet_t;



MESSAGING_CONSUMER(can_telemetry_messaging_consumer, 0, 0, 0, message_flags_dont_send_over_can, transmit_packet, 20);

TELEMETRY_ALLOCATOR(can_telemetry_allocator, 1024);

void can_telemetry_start(void) {
    messaging_consumer_init(&can_telemetry_messaging_consumer);
    telemetry_allocator_init(&can_telemetry_allocator);


    COMPONENT_STATE_UPDATE(avionics_component_can_telemetry, state_ok);
}

static void transmit_can_packet(const can_packet_t* packet, uint16_t packet_id) {
    // TODO: Complete me!!
}

static bool transmit_packet(const telemetry_t* packet, message_metadata_t metadata) {
    (void)metadata;
    if (serusbcfg.usbp->state != USB_ACTIVE)
        return false;
    if (packet->header.origin == local_config.origin) {
        int num_multiples = (packet->header.length + 3) / 4;
        // The multiple tagging overlaps with the actual id!
        // NB: This is a crude check and may miss ids with trailing 0s
        if (num_multiples & packet->id != 0) {
            COMPONENT_STATE_UPDATE(avionics_component_can_telemetry, state_error);
            return true;
        }

        can_packet_t can_packet;
        uint32_t* payload_ptr = packet->payload;
        for (int i = 0; i < num_multiples; i++) {
            can_packet.data = payload_ptr[i];
            transmit_can_packet(&can_packet, packet->id | i);
        }
    }
    return true;
}

static void received_can_packet(const can_packet_t* can_packet) {
    telemetry_t* packet = telemetry_allocator_alloc(&can_telemetry_allocator, 4);
    if (packet == NULL)
        return;
    uint32_t* payload = (uint32_t*)packet->payload;
    *payload = can_packet->data;
    messaging_send(packet, 0);
    chThdYield(); // Ensure that other threads get to run
}

void can_telemetry_receive_thread(void *arg) {
    // TODO: Complete me - receive packets and send them off to received_can_packet
}

void can_telemetry_transmit_thread(void* arg) {
    while (true) {
        messaging_consumer_receive(&can_telemetry_messaging_consumer, true, false);
        chThdYield();
    }
}
