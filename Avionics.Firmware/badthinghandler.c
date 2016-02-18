#include "ch.h"
#include "hal.h"
#include "badthinghandler.h"

#define NO_BEEPING true

volatile bool error_states[ERROR_MAX];

static void setIMUOk(bool ok) {
	// If it is OK we don't set GPIOE_STAT_IMU as the main method will be blinking it
	if (!ok) {
		// SET GPIOE_STAT_NIMU

	}
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

static void beeper(int n, int ontime, int offtime)
{
#if (NO_BEEPING == 0 ? 1:0)
	int i;
	for (i = 0; i<n; i++) {
		palSetPad(GPIOE, GPIOE_STAT_BUZZER);
		chThdSleepMilliseconds(ontime);
		palClearPad(GPIOE, GPIOE_STAT_BUZZER);
		chThdSleepMilliseconds(offtime);
	}
#else
	chThdSleepMilliseconds(n*(offtime + ontime));
#endif

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

		setIMUOk(no_bad_thing);
		setSensorOk(!(error_states[ERROR_ADIS16405] || error_states[ERROR_MPU9250] || error_states[ERROR_ALTIM]));

		if (no_bad_thing) {
			beeper(1, 10, 990);
		}
		else {
			beeper(1, 800, 200);
		}
	}

}

void bthandler_set_error(bthandler_error_t err, bool set) {
	error_states[err] = set;
}
