#include "spalaxconf.h"
#include "adis16405.h"
#include "mpu9250.h"
#include "avionics_config.h"

const EXTConfig extcfg = { {
                     { EXT_CH_MODE_DISABLED, NULL }, /* Pin 0 */
                     { EXT_CH_MODE_DISABLED, NULL }, /* Pin 1 */
                     { EXT_CH_MODE_DISABLED, NULL }, /* Pin 2 */
                     { EXT_CH_MODE_DISABLED, NULL }, /* Pin 3 */
                     { EXT_CH_MODE_DISABLED, NULL }, /* Pin 4 */
                     { EXT_CH_MODE_AUTOSTART | EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOC, adis16405_wakeup }, /* Pin 5 */
                     { EXT_CH_MODE_DISABLED, NULL }, /* Pin 6 */
                     { EXT_CH_MODE_DISABLED, NULL }, /* Pin 7 */
                     { EXT_CH_MODE_DISABLED, NULL }, /* Pin 8 */
                     { EXT_CH_MODE_DISABLED, NULL }, /* Pin 9 */
                     { EXT_CH_MODE_DISABLED, NULL }, /* Pin 10*/
                     { EXT_CH_MODE_AUTOSTART | EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOB, mpu9250_wakeup }, /* Pin 11*/
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

static void update_handler(avionics_component_t component, avionics_component_state_t state, int line) {
    // TODO: Create replacement for this
//    if (state == state_initializing)
//        m3status_set_init(component);
//    else if (state == state_ok)
//        m3status_set_ok(component);
//    else if (state == state_error)
//        m3status_set_error(component, line & 0xFF);
        // This isn't ideal but the full line number will be logged by component_state
}

avionics_config_t local_config = { telemetry_origin_m3imu, update_handler};
