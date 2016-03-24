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
//static WORKING_AREA(waUSB, 1024);
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

const avionics_config_t local_config = {telemetry_origin_imu};

void test_receive(telemetry_t* packet, message_metadata_t flags) {
    (void)packet;
    (void)flags;
}

MESSAGING_PRODUCER(prod1, telemetry_source_state_estimators, telemetry_source_mask_state_estimators, 1024);

MESSAGING_PRODUCER(prod2, telemetry_source_state_estimators, telemetry_source_mask_state_estimators, 1024);

MESSAGING_CONSUMER(consum1, telemetry_source_state_estimators, telemetry_source_mask_state_estimators, 0, 0, test_receive, 20);

MESSAGING_CONSUMER(consum2, telemetry_source_state_estimators, telemetry_source_mask_state_estimators, 0, message_flags_dont_send_over_can, test_receive, 20);

WORKING_AREA(waTest1, 1024);
WORKING_AREA(waTest2, 1024);
WORKING_AREA(waTest3, 1024);
WORKING_AREA(waTest4, 1024);



msg_t dispatchThread(void* producer) {
    if (!messaging_producer_init(producer)) {
        bthandler_set_error(ERROR_CONFIG, true);
        return 1;
    }

    while (true) {
        chThdSleepMilliseconds(2000);
        const char* test = "Hello World";
        //message_producer_t* producer, uint16_t tag, uint8_t* data, uint16_t length
        messaging_producer_send(producer, telemetry_id_state_estimators_quaternion, message_flags_dont_send_over_can, (const uint8_t*)test, 12);
    }
}

msg_t receiveThread(void* consumer) {
    if (!messaging_consumer_init(consumer)) {
        bthandler_set_error(ERROR_CONFIG, true);
        return 1;
    }

    while (true) {
        while (messaging_consumer_receive(consumer, true, false) == messaging_receive_ok);
        chThdSleepMilliseconds(20);
    }
}


int main(void) {
    halInit();
    chSysInit();
    chRegSetThreadName("Main");

    bthandler_reset();
    init_telemetry_allocators();
    init_messaging();

    chThdCreateStatic(waTest1, sizeof(waTest1), NORMALPRIO, dispatchThread, &prod1);
    chThdCreateStatic(waTest2, sizeof(waTest2), NORMALPRIO, dispatchThread, &prod2);

    chThdCreateStatic(waTest3, sizeof(waTest3), NORMALPRIO, receiveThread, &consum1);
    chThdCreateStatic(waTest4, sizeof(waTest4), NORMALPRIO, receiveThread, &consum2);

    //chThdCreateStatic(waMission, sizeof(waMission), NORMALPRIO, mission_thread, NULL);
    chThdCreateStatic(waBadThing, sizeof(waBadThing), NORMALPRIO, bthandler_thread, NULL);
    chThdCreateStatic(waMPU, sizeof(waMPU), NORMALPRIO, mpu9250_thread, NULL);
    //chThdCreateStatic(waMS5611, sizeof(waMS5611), NORMALPRIO, ms5611_thread, NULL);

// #ifdef USE_USB_TELEMETRY
//     chThdCreateStatic(waUSB, sizeof(waUSB), NORMALPRIO, usb_telemetry_thread, NULL);
// #else
//     chThdCreateStatic(waUSB, sizeof(waUSB), NORMALPRIO, serial_console_thread, NULL);
// #endif
    extStart(&EXTD1, &extcfg);

    while (true) {
        chThdSleepMilliseconds(TIME_INFINITE);
    }
}
