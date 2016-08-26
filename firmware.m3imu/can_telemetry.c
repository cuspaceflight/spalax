#include "can_telemetry.h"
#include "component_state.h"
#include "messaging.h"
#include "can_interface.h"

MESSAGING_CONSUMER(can_telemetry_messaging_consumer, 0, 0, message_flags_send_over_can, message_flags_send_over_can, can_send_telemetry, 20);

void can_telemetry_start(void) {
    can_interface_init();
    messaging_consumer_init(&can_telemetry_messaging_consumer);
}

void can_telemetry_transmit_thread(void* arg) {
    (void)arg;
    while (true) {
        messaging_consumer_receive(&can_telemetry_messaging_consumer, true, false);
        chThdYield();
    }
}
