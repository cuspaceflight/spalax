#include "ch.h"
#include "hal.h"
#include "badthinghandler.h"

#define NO_BEEPING

static inline void beep_start(void) {
    #ifndef NO_BEEPING
    palSetPad(GPIOE, GPIOE_STAT_BUZZER);
    #endif
}

static inline void beep_stop(void) {
    #ifndef NO_BEEPING
    palClearPad(GPIOE, GPIOE_STAT_BUZZER);
    #endif
}


static void setSensorOk(bool ok) {
    if (ok) {
        // Set GPIOB_STAT_SENSORS
        // Turn off GPIOE_STAT_NSENSORS
        palSetPad(GPIOB, GPIOB_STAT_SENSORS);
        palClearPad(GPIOE, GPIOE_STAT_NSENSORS);
    }
    else {
        // SET GPIOE_STAT_NSENSORS
        // Turn off GPIOB_STAT_SENSORS
        palClearPad(GPIOB, GPIOB_STAT_SENSORS);
        palSetPad(GPIOE, GPIOE_STAT_NSENSORS);
    }
}

msg_t bthandler_thread(void* arg) {
    (void)arg;
    chRegSetThreadName("BadThingHandler");
    const volatile uint8_t* component_states = component_state_get_states();
    while (true) {

        bool has_error = false;
        bool has_initializing = false;
        for (int i = 0; i < avionics_component_max; i++) {
            avionics_component_state_t state = component_states[i];
            if (state == state_error) {
                has_error = true;
            } else if (state == state_initializing) {
                has_initializing = true;
            }
        }

        setSensorOk(!(component_states[avionics_component_adis16405] == state_error || component_states[avionics_component_mpu9250] == state_error || component_states[avionics_component_ms5611] == state_error));

        if (has_error) {
            palClearPad(GPIOE, GPIOE_STAT_IMU);
            palSetPad(GPIOE, GPIOE_STAT_NIMU);
            beep_start();
            chThdSleepMilliseconds(500);
            palClearPad(GPIOE, GPIOE_STAT_NIMU);
            beep_stop();
            chThdSleepMilliseconds(500);
            palClearPad(GPIOE, GPIOE_STAT_NIMU);
        } else if (has_initializing) {
            palSetPad(GPIOE, GPIOE_STAT_IMU);
            palSetPad(GPIOE, GPIOE_STAT_NIMU);
            beep_start();
            chThdSleepMilliseconds(500);
            beep_stop();
            chThdSleepMilliseconds(500);
        } else {
            palSetPad(GPIOE, GPIOE_STAT_IMU);
            palClearPad(GPIOE, GPIOE_STAT_NIMU);
            beep_start();
            chThdSleepMilliseconds(20);
            beep_stop();
            chThdSleepMilliseconds(480);
            palClearPad(GPIOE, GPIOE_STAT_IMU);
            chThdSleepMilliseconds(500);
        }
    }

}
