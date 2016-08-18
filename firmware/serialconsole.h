// serialconsole.h contains public declarations for serial-over-USB
// functionality.
#pragma once
#ifndef SERIAL_CONSOLE__H
#define SERIAL_CONSOLE__H

// serial_console_thread initialises a USB serial interface and wires it up to
// the serial console.
msg_t serial_console_thread(void *arg);

bool serial_console_is_active(void);

#endif // SERIAL_CONSOLE__H
