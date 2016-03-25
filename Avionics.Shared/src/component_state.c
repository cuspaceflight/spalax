#include "component_state.h"
#include "messaging.h"
#include "compilermacros.h"

STATIC_ASSERT(avionics_component_max < UINT8_MAX, too_many_avionics_components);
static volatile uint8_t component_states[avionics_component_max];

typedef struct component_state_update_t {
    uint8_t component;
    uint8_t state;
    uint16_t line_number;
} component_state_update_t;

STATIC_ASSERT(sizeof(component_state_update_t) == 4, component_state_update_invalid_size);

MESSAGING_PRODUCER(messaging_producer, telemetry_source_component_state, telemetry_source_mask_component_state, 128);

void component_state_init(void) {
    for (int i = 0; i < avionics_component_max; i++)
        component_states[i] = state_uninitialized;

    messaging_producer_init(&messaging_producer);
}

void component_state_update(avionics_component_t component, avionics_component_state_t state, uint16_t line_number) {
    component_states[component] = state;

    if (component != avionics_component_messaging || state != state_error || component_states[component] != state) {
        // The if statement protects against potential error loops when
        // there is a problem in the messaging system:
        // error generated -> we send a packet -> an error is generated...
        component_state_update_t packet;
        packet.component = component;
        packet.state = state;
        packet.line_number = line_number;

        messaging_producer_send(&messaging_producer, telemetry_id_component_state_update, 0, (uint8_t*)&packet, sizeof(component_state_update));
    }

    if (state == state_error && local_config.error_handler != NULL)
        local_config.error_handler(component, line_number);
}

avionics_component_state_t component_state_get(avionics_component_t component) {
    return component_states[component];
}
