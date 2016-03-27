#include "component_state.h"
#include "messaging.h"
#include "platform.h"

STATIC_ASSERT(avionics_component_max < UINT8_MAX, too_many_avionics_components);
static volatile uint8_t component_states[avionics_component_max];

typedef struct component_state_update_t {
    uint8_t component;
    uint8_t state;
    uint16_t line_number;
} component_state_update_t;

STATIC_ASSERT(sizeof(component_state_update_t) == 4, component_state_update_invalid_size);

MESSAGING_PRODUCER(messaging_producer, telemetry_source_component_state, telemetry_source_mask_component_state, 128);

static volatile bool is_started = false;
static volatile bool messaging_enabled = false;

void component_state_start(void) {
    for (int i = 0; i < avionics_component_max; i++)
        component_states[i] = state_uninitialized;

    memory_barrier_release();

    is_started = true;
}

bool component_state_started(void) {
    bool is_started_local = is_started;
    memory_barrier_acquire();
    return is_started_local;
}

void component_state_register_with_messaging(void) {
    messaging_producer_init(&messaging_producer);
    memory_barrier_release();
    messaging_enabled = true;
}

void component_state_update(avionics_component_t component, avionics_component_state_t state, uint16_t line_number) {
    component_states[component] = state;

    bool messaging_enabled_local = messaging_enabled;
    memory_barrier_acquire();

    if (messaging_enabled_local && (component != avionics_component_messaging || state != state_error || component_states[component] != state)) {
        // The if statement protects against potential error loops when
        // there is a problem in the messaging system:
        // error generated -> we send a packet -> an error is generated...
        component_state_update_t packet;
        packet.component = component;
        packet.state = state;
        packet.line_number = line_number;

        messaging_producer_send(&messaging_producer, telemetry_id_component_state_update, 0, (uint8_t*)&packet, sizeof(component_state_update_t));
    }

    if (state == state_error && local_config.error_handler != NULL)
        local_config.error_handler(component, line_number);
}

const volatile uint8_t* component_state_get_states(void) {
    return component_states;
}
