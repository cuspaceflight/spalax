#include "ch.h"
#include "hal.h"
#include "state_estimate.h"
#include "mpu9250.h"
#include "ms5611.h"
#include "badthinghandler.h"
#include "serialconsole.h"
#include "messaging.h"
#include "usb_telemetry.h"
#include "telemetry_allocator.h"

#define USE_USB_TELEMETRY

//static WORKING_AREA(waMission, 1024);

static WORKING_AREA(waMPU, 2048);
static WORKING_AREA(waBadThing, 1024);
static WORKING_AREA(waUSB, 1024);
//static WORKING_AREA(waMS5611, 768);

/*
 * Set up pin change interrupts for the various sensors that react to them.
 */
static const EXTConfig extcfg = {{
    {EXT_CH_MODE_DISABLED, NULL}, /* Pin 0 */
    {EXT_CH_MODE_DISABLED, NULL}, /* Pin 1 */
    {EXT_CH_MODE_DISABLED, NULL}, /* Pin 2 */
    {EXT_CH_MODE_DISABLED, NULL}, /* Pin 3 */
    {EXT_CH_MODE_DISABLED, NULL}, /* Pin 4 */
    {EXT_CH_MODE_DISABLED, NULL}, /* Pin 5 */
    {EXT_CH_MODE_DISABLED, NULL}, /* Pin 6 */
    {EXT_CH_MODE_DISABLED, NULL}, /* Pin 7 */
    {EXT_CH_MODE_DISABLED, NULL}, /* Pin 8 */
    {EXT_CH_MODE_DISABLED, NULL}, /* Pin 9 */
    {EXT_CH_MODE_DISABLED, NULL}, /* Pin 10*/
    {EXT_CH_MODE_DISABLED, NULL}, /* Pin 11*/
    {EXT_CH_MODE_DISABLED, NULL}, /* Pin 12*/
    {EXT_CH_MODE_DISABLED, NULL}, /* Pin 13*/
    {EXT_CH_MODE_DISABLED, NULL}, /* Pin 14*/
    {EXT_CH_MODE_DISABLED, NULL}, /* Pin 15*/
    {EXT_CH_MODE_DISABLED, NULL}, /* Pin 16*/
    {EXT_CH_MODE_DISABLED, NULL}, /* Pin 17*/
    {EXT_CH_MODE_DISABLED, NULL}, /* Pin 18*/
    {EXT_CH_MODE_DISABLED, NULL}, /* Pin 19*/
    {EXT_CH_MODE_DISABLED, NULL}, /* Pin 20*/
    {EXT_CH_MODE_DISABLED, NULL}, /* Pin 21*/
    {EXT_CH_MODE_DISABLED, NULL}  /* Pin 22*/
}};

MESSAGING_PRODUCER(prod1, telemetry_source_state_estimators, telemetry_source_mask_state_estimators, 1024);

void test_receive(telemetry_t* packet) {
    (void)packet;
}

MESSAGING_CONSUMER(consum1, telemetry_source_state_estimators, telemetry_source_mask_state_estimators, test_receive, 20);





int main(void) {
    halInit();
    chSysInit();
    chRegSetThreadName("Main");

    bthandler_reset();
    init_telemetry_allocators();
    init_messaging();

    if (!messaging_producer_init(&prod1)) {
        bthandler_set_error(ERROR_CONFIG, true);
    }

    if (!messaging_consumer_init(&consum1)) {
        bthandler_set_error(ERROR_CONFIG, true);
    }

    const char* test = "Hello World";
    //message_producer_t* producer, uint16_t tag, uint8_t* data, uint16_t length
    messaging_producer_send(&prod1, telemetry_id_state_estimators_quaternion, (const uint8_t*)test, 12);

    messaging_consumer_receive(&consum1, true, false);

    //chThdCreateStatic(waMission, sizeof(waMission), NORMALPRIO, mission_thread, NULL);
    chThdCreateStatic(waBadThing, sizeof(waBadThing), NORMALPRIO, bthandler_thread, NULL);
    chThdCreateStatic(waMPU, sizeof(waMPU), NORMALPRIO, mpu9250_thread, NULL);
    //chThdCreateStatic(waMS5611, sizeof(waMS5611), NORMALPRIO, ms5611_thread, NULL);

#ifdef USE_USB_TELEMETRY
    chThdCreateStatic(waUSB, sizeof(waUSB), NORMALPRIO, usb_telemetry_thread, NULL);
#else
    chThdCreateStatic(waUSB, sizeof(waUSB), NORMALPRIO, serial_console_thread, NULL);
#endif
    extStart(&EXTD1, &extcfg);

    while (true) {
        chThdSleepMilliseconds(TIME_INFINITE);
    }
}
