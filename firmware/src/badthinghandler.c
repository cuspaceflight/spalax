#include "ch.h"
#include "hal.h"
#include "badthinghandler.h"
#include "spalaxconf.h"

static inline void beep_start(void) {
    #if BAD_THING_HANDLER_BEEP_ENABLED
    palSetPad(BAD_THING_HANDLER_BUZZER_PORT, BAD_THING_HANDLER_BUZZER_PIN);
    #endif
}

static inline void beep_stop(void) {
    #if BAD_THING_HANDLER_BEEP_ENABLED
    palClearPad(BAD_THING_HANDLER_BUZZER_PORT, BAD_THING_HANDLER_BUZZER_PIN);
    #endif
}

#if BAD_THIND_HANDLER_HAS_SENSOR_LEDS
static void setSensorOk(bool ok) {
    if (ok) {
        palSetPad(BAD_THING_HANDLER_SENSOR_OK_LED_PORT, BAD_THING_HANDLER_SENSOR_OK_LED_PIN);
        palClearPad(BAD_THING_HANDLER_SENSOR_NOK_LED_PORT, BAD_THING_HANDLER_SENSOR_NOK_LED_PIN);
    }
    else {
        palClearPad(BAD_THING_HANDLER_SENSOR_OK_LED_PORT, BAD_THING_HANDLER_SENSOR_OK_LED_PIN);
        palSetPad(BAD_THING_HANDLER_SENSOR_NOK_LED_PORT, BAD_THING_HANDLER_SENSOR_NOK_LED_PIN);
    }
}
#endif

void bthandler_thread(void* arg) {
    (void)arg;
    chRegSetThreadName("BadThingHandler");

    while (true) {
        avionics_component_state_t state = component_state_get_overall_state();
#if BAD_THIND_HANDLER_HAS_SENSOR_LEDS
        const volatile uint8_t* component_states = component_state_get_states();
        setSensorOk(!(component_states[avionics_component_adis16405] == state_error || component_states[avionics_component_mpu9250] == state_error || component_states[avionics_component_ms5611] == state_error));
#endif

        if (state == state_error) {
            palClearPad(BAD_THING_HANDLER_OK_LED_PORT, BAD_THING_HANDLER_OK_LED_PIN);
            palSetPad(BAD_THING_HANDLER_NOK_LED_PORT, BAD_THING_HANDLER_NOK_LED_PIN);
            beep_start();
            chThdSleepMilliseconds(500);
            palClearPad(BAD_THING_HANDLER_NOK_LED_PORT, BAD_THING_HANDLER_NOK_LED_PIN);
            beep_stop();
            chThdSleepMilliseconds(500);
            palClearPad(BAD_THING_HANDLER_NOK_LED_PORT, BAD_THING_HANDLER_NOK_LED_PIN);
        } else if (state == state_initializing) {
            palSetPad(BAD_THING_HANDLER_OK_LED_PORT, BAD_THING_HANDLER_OK_LED_PIN);
            palSetPad(BAD_THING_HANDLER_NOK_LED_PORT, BAD_THING_HANDLER_NOK_LED_PIN);
            beep_start();
            chThdSleepMilliseconds(500);
            beep_stop();
            chThdSleepMilliseconds(500);
        } else if (state == state_ok) {
            palSetPad(BAD_THING_HANDLER_OK_LED_PORT, BAD_THING_HANDLER_OK_LED_PIN);
            palClearPad(BAD_THING_HANDLER_NOK_LED_PORT, BAD_THING_HANDLER_NOK_LED_PIN);
            beep_start();
            chThdSleepMilliseconds(20);
            beep_stop();
            chThdSleepMilliseconds(480);
            palClearPad(BAD_THING_HANDLER_OK_LED_PORT, BAD_THING_HANDLER_OK_LED_PIN);
            chThdSleepMilliseconds(500);
        } else {
            palSetPad(BAD_THING_HANDLER_OK_LED_PORT, BAD_THING_HANDLER_OK_LED_PIN);
            palSetPad(BAD_THING_HANDLER_NOK_LED_PORT, BAD_THING_HANDLER_NOK_LED_PIN);
            beep_start();
            chThdSleepMilliseconds(500);
            palClearPad(BAD_THING_HANDLER_NOK_LED_PORT, BAD_THING_HANDLER_NOK_LED_PIN);
            palClearPad(BAD_THING_HANDLER_OK_LED_PORT, BAD_THING_HANDLER_OK_LED_PIN);
            beep_stop();
            chThdSleepMilliseconds(500);
        }
    }

}
