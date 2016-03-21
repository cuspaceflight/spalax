#include "ch.h"
#include "hal.h"
#include "badthinghandler.h"

#define NO_BEEPING

volatile bool error_states[ERROR_MAX];

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

void bthandler_reset(void) {
    for (int i = 0; i < ERROR_MAX; i++) {
        error_states[i] = false;
    }
}

msg_t bthandler_thread(void* arg) {
    (void)arg;
    chRegSetThreadName("BadThingHandler");
    while (true) {
        bool no_bad_thing = true;
        for (int i = 0; i < ERROR_MAX; i++) {
            if (error_states[i]) {
                no_bad_thing = false;
            }
        }

        setSensorOk(!(error_states[ERROR_ADIS16405] || error_states[ERROR_MPU9250] || error_states[ERROR_ALTIM]));

        if (no_bad_thing) {
            palSetPad(GPIOE, GPIOE_STAT_IMU);
            palClearPad(GPIOE, GPIOE_STAT_NIMU);
#ifndef NO_BEEPING
            palSetPad(GPIOE, GPIOE_STAT_BUZZER);
#endif
            chThdSleepMilliseconds(20);
#ifndef NO_BEEPING
            palClearPad(GPIOE, GPIOE_STAT_BUZZER);
#endif
            chThdSleepMilliseconds(480);
            palClearPad(GPIOE, GPIOE_STAT_IMU);
            chThdSleepMilliseconds(500);
        }
        else {
            palSetPad(GPIOE, GPIOE_STAT_IMU);
            palClearPad(GPIOE, GPIOE_STAT_NIMU);
#ifndef NO_BEEPING
            palSetPad(GPIOE, GPIOE_STAT_BUZZER);
#endif
            chThdSleepMilliseconds(500);
#ifndef NO_BEEPING
            palClearPad(GPIOE, GPIOE_STAT_BUZZER);
#endif
            chThdSleepMilliseconds(500);
            palClearPad(GPIOE, GPIOE_STAT_NIMU);
        }
    }

}

void bthandler_set_error(bthandler_error_t err, bool set) {
    error_states[err] = set;
}
