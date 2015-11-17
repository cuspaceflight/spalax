#include "ch.h"
#include "hal.h"
#include "state_estimate.h"

//static WORKING_AREA(waMission, 1024);


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

int main(void) {
    halInit();
    chSysInit();
    chRegSetThreadName("Main");

    // Temporary - here to make sure linker configured correctly
    state_estimate_t linktest;


    reset_state_estimate(&linktest);
    linktest.pos[0] = 1;
    linktest.pos[1] = 2;
    linktest.pos[2] = 3;
    print_state_estimate(&linktest);
    
    //chThdCreateStatic(waMission, sizeof(waMission), NORMALPRIO, mission_thread, NULL);

    extStart(&EXTD1, &extcfg);

    while (TRUE) {
        palSetPad(GPIOD, GPIOD_IMU_GRN);
        chThdSleepMilliseconds(500);
        palClearPad(GPIOD, GPIOD_IMU_GRN);
        chThdSleepMilliseconds(500);
    }
}
