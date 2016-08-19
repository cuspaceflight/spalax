#ifndef USB_TELEMETRY_H
#define USB_TELEMETRY_H
#include <stdint.h>

void usb_telemetry_start(void);

void usb_telemetry_receive_thread(void *arg);

void usb_telemetry_transmit_thread(void* arg);

#endif /* USB_TELEMETRY_H */
