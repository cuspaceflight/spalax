#ifndef COMPONENT_STATE_H
#define COMPONENT_STATE_H
#include "avionics_config.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    state_ok = 0,
    state_uninitialized = 1,
    state_initializing = 2,
    state_error = 3
} avionics_component_state_t;

#define COMPONENT_STATE_UPDATE(component, state) component_state_update(component, state, __LINE__)

// Should be called after the messaging system has been initialized
void component_state_start(void);

bool component_state_started(void);

// The macro above is the recommended way to call this - it will add the line number for you
void component_state_update(avionics_component_t component, avionics_component_state_t state, uint16_t line_number);

const volatile uint8_t* component_state_get_states(void);

#ifdef __cplusplus
}
#endif

#endif /* COMPONENT_STATE_H */
