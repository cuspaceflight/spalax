#include "ch.h"
#include "hal.h"
#include "state_estimate.h"
#include "mpu9250.h"
#include "ms5611.h"
#include "adis16405.h"
#include "badthinghandler.h"
#include "serialconsole.h"
#include "messaging.h"
#include "usb_telemetry.h"
#include "telemetry_allocator.h"
#include "checksum.h"
#include "avionics_config.h"
#include "state_estimate.h"

#define USE_USB_TELEMETRY

static WORKING_AREA(waMPU, 1024);
static WORKING_AREA(waBadThing, 1024);
static WORKING_AREA(waMS5611, 768);
static WORKING_AREA(waADIS, 1024);
static WORKING_AREA(waStateEstimation, 2048);

#ifdef USE_USB_TELEMETRY
static WORKING_AREA(waUSBTransmit, 512);
static WORKING_AREA(waUSBReceive, 512);
#else
static WORKING_AREA(waUSB, 1024);
#endif


/*
 * Set up pin change interrupts for the various sensors that react to them.
 */
static const EXTConfig extcfg = { {
					  { EXT_CH_MODE_DISABLED, NULL }, /* Pin 0 */
					  { EXT_CH_MODE_AUTOSTART | EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOA, mpu9250_wakeup }, /* Pin 1 */
					  { EXT_CH_MODE_DISABLED, NULL }, /* Pin 2 */
					  { EXT_CH_MODE_DISABLED, NULL }, /* Pin 3 */
					  { EXT_CH_MODE_DISABLED, NULL }, /* Pin 4 */
					  { EXT_CH_MODE_DISABLED, NULL }, /* Pin 5 */
					  { EXT_CH_MODE_DISABLED, NULL }, /* Pin 6 */
					  { EXT_CH_MODE_DISABLED, NULL }, /* Pin 7 */
					  { EXT_CH_MODE_DISABLED, NULL }, /* Pin 8 */
					  { EXT_CH_MODE_DISABLED, NULL }, /* Pin 9 */
					  { EXT_CH_MODE_DISABLED, NULL }, /* Pin 10*/
					  { EXT_CH_MODE_DISABLED, NULL }, /* Pin 11*/
					  { EXT_CH_MODE_DISABLED, NULL }, /* Pin 12*/
					  { EXT_CH_MODE_DISABLED, NULL }, /* Pin 13*/
					  { EXT_CH_MODE_DISABLED, NULL }, /* Pin 14*/
					  { EXT_CH_MODE_DISABLED, NULL }, /* Pin 15*/
					  { EXT_CH_MODE_DISABLED, NULL }, /* Pin 16*/
					  { EXT_CH_MODE_DISABLED, NULL }, /* Pin 17*/
					  { EXT_CH_MODE_DISABLED, NULL }, /* Pin 18*/
					  { EXT_CH_MODE_DISABLED, NULL }, /* Pin 19*/
					  { EXT_CH_MODE_DISABLED, NULL }, /* Pin 20*/
					  { EXT_CH_MODE_DISABLED, NULL }, /* Pin 21*/
					  { EXT_CH_MODE_DISABLED, NULL } /* Pin 22*/
				  } };

const avionics_config_t local_config = { telemetry_origin_imu, NULL };

int main(void) {
	halInit();
	chSysInit();
	chRegSetThreadName("Main");

	component_state_start();
	checksum_init();
	telemetry_allocator_start();
	messaging_start();



	//chThdCreateStatic(waMission, sizeof(waMission), NORMALPRIO, mission_thread, NULL);
	chThdCreateStatic(waBadThing, sizeof(waBadThing), NORMALPRIO, bthandler_thread, NULL);
	//chThdCreateStatic(waMPU, sizeof(waMPU), NORMALPRIO, mpu9250_thread, NULL);
	chThdCreateStatic(waMS5611, sizeof(waMS5611), NORMALPRIO, ms5611_thread, NULL);
	chThdCreateStatic(waADIS, sizeof(waADIS), NORMALPRIO, adis16405_thread, NULL);
    chThdCreateStatic(waStateEstimation, sizeof(waStateEstimation), NORMALPRIO, state_estimate_thread, NULL);

    #ifdef USE_USB_TELEMETRY
        usb_telemetry_start();

    	chThdCreateStatic(waUSBReceive, sizeof(waUSBReceive), NORMALPRIO, usb_telemetry_receive_thread, NULL);
    	chThdCreateStatic(waUSBTransmit, sizeof(waUSBTransmit), NORMALPRIO, usb_telemetry_transmit_thread, NULL);
    #else
    	chThdCreateStatic(waUSB, sizeof(waUSB), NORMALPRIO, serial_console_thread, NULL);
    #endif

	extStart(&EXTD1, &extcfg);

	while (true) {
		chThdSleepMilliseconds(TIME_INFINITE);
	}
}
