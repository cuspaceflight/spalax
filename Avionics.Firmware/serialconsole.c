#include "ch.h"
#include "hal.h"
#include "shell.h"

#include "usbcfg.h"
#include "serialconsole.h"
#include "compilermacros.h"

/* Virtual serial port over USB.*/
SerialUSBDriver SDU1;

static const ShellCommand commands[] = {
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SDU1,
  commands
};

#define SHELL_WA_SIZE   THD_WA_SIZE(2048)

msg_t serial_console_thread(COMPILER_UNUSED_ARG(void *arg)) {
    Thread *shelltp = NULL;

    chRegSetThreadName("Serial Console");

    // Initializes a serial-over-USB CDC driver.
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    // Activates the USB driver and then the USB bus pull-up on D+.
    // Note, a delay is inserted in order to not have to disconnect the cable
    // after a reset.
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1000);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    while(TRUE) {
        if (!shelltp) {
            if (SDU1.config->usbp->state == USB_ACTIVE) {
                /* Spawns a new shell.*/
                shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
            }
        }
        else {
            /* If the previous shell exited.*/
            if (chThdTerminated(shelltp)) {
              /* Recovers memory of the previous shell.*/
              chThdRelease(shelltp);
              shelltp = NULL;
            }
        }
        chThdSleepMilliseconds(500);
    }
}

bool serial_console_is_active(void) {
    return SDU1.config->usbp->state == USB_ACTIVE;
}
