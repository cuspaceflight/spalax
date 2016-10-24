#ifndef CAN_TELEMETRY_H
#define CAN_TELEMETRY_H
#include <stdint.h>

void can_telemetry_start(void);

void can_telemetry_transmit_thread(void* arg);

#endif /* CAN_TELEMETRY_H */
