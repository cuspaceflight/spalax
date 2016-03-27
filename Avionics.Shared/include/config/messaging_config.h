#ifndef MESSAGING_CONFIG_H
#define MESSAGING_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

// We use the messaging system's metadata to store flags for interface drivers
typedef enum {
    message_flags_dont_send_over_can = 1 << 0,
    message_flags_dont_send_over_usb = 1 << 1,
    message_flags_dont_send_over_rs232 = 1 << 2,
    message_flags_dont_send_over_radio = 1 << 3,
} message_flags_t;

#ifdef __cplusplus
}
#endif

#endif /* MESSAGING_CONFIG_H */
