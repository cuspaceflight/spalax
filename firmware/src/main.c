#include <messaging_all.h>
#include <util/board_config.h>
#include <file_telemetry.h>
#include "mpu9250.h"
#include "ms5611.h"
#include "adis16405.h"
#include "ublox.h"
#include "badthinghandler.h"
#include "spalaxconf.h"
#include "util/board_config.h"

static THD_WORKING_AREA(waMPU, 256);
static THD_WORKING_AREA(waBadThing, 256);
static THD_WORKING_AREA(waMS5611, 256);
static THD_WORKING_AREA(waADIS, 256);
static THD_WORKING_AREA(waGPS, 1024);

#if BUILD_STATE_ESTIMATORS
#include <state/state_estimate.h>
// This is possibly more than it needs - but can be tweaked later
// It will need substantially more than other threads
static THD_WORKING_AREA(waStateEstimators, 4086 * 2);
#endif

int main(void) {
    halInit();
    chSysInit();
    chRegSetThreadName("Main");

    messaging_all_start();

    extStart(&EXTD1, &extcfg);

    board_config_t *config = getBoardConfig();

    chThdCreateStatic(waBadThing, sizeof(waBadThing), NORMALPRIO, bthandler_thread, NULL);

#if FILE_TELEMETRY_OUTPUT_ENABLED
    if (config->has_sdcard)
        file_telemetry_output_start("log.m3tel", false);
#endif

    if (config->has_ms5611)
        chThdCreateStatic(waMS5611, sizeof(waMS5611), NORMALPRIO, ms5611_thread, NULL);

    if (config->has_mpu9250)
        chThdCreateStatic(waMPU, sizeof(waMPU), NORMALPRIO, mpu9250_thread, NULL);

    if (config->has_gps)
        chThdCreateStatic(waGPS, sizeof(waGPS), NORMALPRIO, ublox_thread, NULL);

    if (config->has_adis)
        chThdCreateStatic(waADIS, sizeof(waADIS), NORMALPRIO, adis16405_thread, NULL);
#if BUILD_STATE_ESTIMATORS
    if (config->run_state_estimators)
        chThdCreateStatic(waStateEstimators, sizeof(waStateEstimators), NORMALPRIO, state_estimate_thread, NULL);
#endif
    while (true) {
        chThdSleepMilliseconds(TIME_INFINITE);
    }
}
