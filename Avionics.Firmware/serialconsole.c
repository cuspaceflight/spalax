#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "shell.h"

#include "usbcfg.h"
#include "serialconsole.h"
#include "compilermacros.h"

static void cmd_threads(BaseSequentialStream *chp,
                        int argc, COMPILER_UNUSED_ARG(char *argv[]))
{
    static const char *states[] = {THD_STATE_NAMES};
    uint64_t busy = 0, total = 0;
    Thread *tp;

    if (argc > 0) {
        chprintf(chp, "Usage: threads\r\n");
        return;
    }

    chprintf(chp,
        "name        |addr    |stack   |free|prio|refs|state    |time\r\n");
    chprintf(chp,
        "------------|--------|--------|----|----|----|---------|--------\r\n");
    tp = chRegFirstThread();
    do {
        chprintf(chp, "%12s|%.8lx|%.8lx|%4lu|%4lu|%4lu|%9s|%lu\r\n",
                 chRegGetThreadName(tp),
                 (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
                 (uint32_t)tp->p_ctx.r13 - (uint32_t)tp->p_stklimit,
                 (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
                 states[tp->p_state], (uint32_t)tp->p_time);
        if(tp->p_prio != 1) {
            busy += tp->p_time;
        }
        total += tp->p_time;
        tp = chRegNextThread(tp);
    } while (tp != NULL);
    chprintf(chp, "CPU Usage: %ld%%\r\n", busy*100/total);
}

/* Virtual serial port over USB.*/
SerialUSBDriver SDU1;

static const ShellCommand commands[] = {
    {"threads", cmd_threads},
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
