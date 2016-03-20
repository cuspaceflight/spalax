#include "usb_telemetry.h"
#include "ch.h"
#include "hal.h"
#include "usbcfg.h"

/* Virtual serial port over USB.*/
extern SerialUSBDriver SDU1;


msg_t usb_telemetry_thread(void *arg) {
	(void)arg;
	chRegSetThreadName("USB Telemetry");

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

	BaseChannel *pChn;
	size_t numBytes;
	uint8_t msg[8];

	while (true) {
		pChn = serusbcfg.usbp->state == USB_ACTIVE ? (BaseChannel*)&SDU1 : NULL;
		if (pChn) {
			numBytes = chnReadTimeout(pChn, msg, 8, TIME_INFINITE);
			if (numBytes != 0)
				chnWrite(pChn, msg, numBytes);
		}
		chThdSleepMilliseconds(1000);
	}
}
