#include "usb_telemetry.h"
#include "ch.h"
#include "hal.h"
#include "usbcfg.h"
#include "serial_interface.h"
#include "messaging.h"
#include "avionics_config.h"
#include "platform.h"

/* Virtual serial port over USB.*/
// Defined in usbcfg.c
extern SerialUSBDriver SDU1;

static uint8_t stream_get(void) {
    return sdGet(&SDU1);
}

static bool stream_put(uint8_t byte) {
    return sdPutTimeout(&SDU1, byte, TIME_IMMEDIATE) == Q_OK;
}

SERIAL_INTERFACE(serial_interface, stream_get, stream_put, NULL, 1024);

static bool transmit_packet(const telemetry_t* packet, message_metadata_t metadata);

MESSAGING_CONSUMER(usb_telemetry_messaging_consumer, 0, 0, 0, message_flags_dont_send_over_usb, transmit_packet, 20);

static bool transmit_packet(const telemetry_t* packet, message_metadata_t metadata) {
    (void)metadata;
    if (serusbcfg.usbp->state != USB_ACTIVE)
        return false;
    if (packet->header.origin == local_config.origin) {
        if (!serial_interface_send_packet(&serial_interface, packet)) {
            // This will cause the transmit thread to back off for a second
            // This occurs when the receiver's input buffer is full
            // Either we are writing to it too fast or nothing is reading it
            return false;
        }
    }
    return true;
}

static volatile bool is_initialised = false;

void usb_telemetry_start(void) {
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    // Activates the USB driver and then the USB bus pull-up on D+.
    // Note, a delay is inserted in order to not have to disconnect the cable
    // after a reset.
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1000);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    messaging_consumer_init(&usb_telemetry_messaging_consumer);
    serial_interface_init(&serial_interface);
    messaging_pause_consumer(&usb_telemetry_messaging_consumer, true);

    memory_barrier_release();
    is_initialised = true;
}

int32_t usb_telemetry_transmit_thread(void *arg) {
    (void)arg;
    chRegSetThreadName("USB Telemetry Transmit");
    // Wait for setup
    while (true) {
        memory_barrier_acquire();
        if (is_initialised)
            break;
        chThdSleepMilliseconds(20);
    }


    while (true) {
        while (serusbcfg.usbp->state != USB_ACTIVE) {
            messaging_pause_consumer(&usb_telemetry_messaging_consumer, true);
            chThdSleepMilliseconds(1000);
        }
        messaging_resume_consumer(&usb_telemetry_messaging_consumer);

        if (messaging_consumer_receive(&usb_telemetry_messaging_consumer, true, false) == messaging_receive_callback_error) {
            messaging_pause_consumer(&usb_telemetry_messaging_consumer, true);
            chThdSleepMilliseconds(1000);
        } else {
            chThdYield();
        }
    }
}

int32_t usb_telemetry_receive_thread(void* arg) {
    (void)arg;
    chRegSetThreadName("USB Telemetry Receive");
    // Wait for setup
    while (true) {
        memory_barrier_acquire();
        if (is_initialised)
            break;
        chThdSleepMilliseconds(20);
    }

    while (true) {
        if (serusbcfg.usbp->state == USB_ACTIVE) {
            telemetry_t* packet = serial_interface_next_packet(&serial_interface);
            if (packet != NULL)
                messaging_send(packet, 0);
            chThdYield(); // Ensure that other threads get to run
        } else {
            chThdSleepMilliseconds(500);
        }
    }
}
