#include "ch.h"
#include "hal.h"
#include "state/state_estimate.h"
#include "mpu9250.h"
#include "ms5611.h"
#include "adis16405.h"
#include "badthinghandler.h"
#include "messaging.h"
#include "usb_telemetry.h"
#include "checksum.h"
#include "util/calibration.h"
#include "spalaxconf.h"
#include "util/board_config.h"

static THD_WORKING_AREA(waMPU, 1024);
static THD_WORKING_AREA(waBadThing, 1024);
static THD_WORKING_AREA(waMS5611, 768);
static THD_WORKING_AREA(waCalibration, 512);
static THD_WORKING_AREA(waADIS, 1024);
static THD_WORKING_AREA(waStateEstimation, 2048);

static THD_WORKING_AREA(waUSBTransmit, 512);
static THD_WORKING_AREA(waUSBReceive, 512);
//static THD_WORKING_AREA(waCanTelemetry, 512);


/*
 * Set up pin change interrupts for the various sensors that react to them.
 */

int main(void) {
	halInit();
	chSysInit();
	chRegSetThreadName("Main");

	component_state_start();
	checksum_init();
	telemetry_allocator_start();
	messaging_start();

	extStart(&EXTD1, &extcfg);

	board_config_t* config = getBoardConfig();

	chThdCreateStatic(waBadThing, sizeof(waBadThing), NORMALPRIO, bthandler_thread, NULL);
	chThdCreateStatic(waMPU, sizeof(waMPU), NORMALPRIO, mpu9250_thread, NULL);
	chThdCreateStatic(waMS5611, sizeof(waMS5611), NORMALPRIO, ms5611_thread, NULL);
	chThdCreateStatic(waCalibration, sizeof(waCalibration), NORMALPRIO, calibration_thread, NULL);

	if (config->has_adis)
		chThdCreateStatic(waADIS, sizeof(waADIS), NORMALPRIO, adis16405_thread, NULL);
    chThdCreateStatic(waStateEstimation, sizeof(waStateEstimation), NORMALPRIO, state_estimate_thread, NULL);

    usb_telemetry_start();

    chThdCreateStatic(waUSBReceive, sizeof(waUSBReceive), NORMALPRIO, usb_telemetry_receive_thread, NULL);
    chThdCreateStatic(waUSBTransmit, sizeof(waUSBTransmit), NORMALPRIO, usb_telemetry_transmit_thread, NULL);

	while (true) {
		chThdSleepMilliseconds(TIME_INFINITE);
	}
}