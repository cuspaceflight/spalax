#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H
#include <stdint.h>
#include "telemetry_allocator.h"
#include "messaging.h"

#ifdef __cplusplus
extern "C" {
#endif

void can_interface_init(void);

bool can_send_telemetry(const telemetry_t* packet, message_metadata_t metadata);

void can_recv(uint16_t msg_id, bool can_rtr, uint8_t* data, uint8_t datalen);

#ifdef __cplusplus
}
#endif

#endif /* CAN_INTERFACE_H */
