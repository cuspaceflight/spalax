#include "can_interface.h"
#include "messaging.h"
#include <string.h>
#include <avionics_config.h>
#include <component_state.h>
#include "mpu9250_config.h"
#include <adis16405_config.h>
#include <state_estimate.h>
#include <calibration.h>

typedef struct {
	int16_t base_id;
	uint8_t size_in_bytes;
	uint8_t num_bits;
	uint8_t data_buffer[32];
	bool is_valid[4];
	uint8_t size_in_packets;
	uint16_t seqno_mask;
	telemetry_origin_t current_origin;
} multipacket_message_t;


static const int num_multipacket_messages = 6;
static multipacket_message_t multipacket_messages[6] = {
	{.base_id = telemetry_id_mpu9250_data, .size_in_bytes = sizeof(mpu9250_data_t), .num_bits = 2},
	{.base_id = telemetry_id_mpu9250_config, .size_in_bytes = sizeof(mpu9250_config_t), .num_bits = 2},
	{.base_id = telemetry_id_adis16405_config, .size_in_bytes = sizeof(adis16405_config_t),.num_bits = 2},
	{.base_id = telemetry_id_adis16405_data, .size_in_bytes = sizeof(adis16405_data_t),.num_bits = 2},
	{.base_id = telemetry_id_state_estimate_data, .size_in_bytes = sizeof(state_estimate_t),.num_bits = 2},
	{ .base_id = telemetry_id_calibration_magno_data,.size_in_bytes = sizeof(magno_calibration_data_t),.num_bits = 2 }
};

static void resetMultipacketMessage(multipacket_message_t* msg) {
	msg->size_in_packets = (msg->size_in_bytes + 7) / 8;
	if (msg->size_in_packets > 4) {
		COMPONENT_STATE_UPDATE(avionics_component_can_telemetry, state_error);
		return;
	}
	msg->seqno_mask = (1 << msg->num_bits) - 1;
	if (msg->seqno_mask < msg->size_in_packets - 1) {
		COMPONENT_STATE_UPDATE(avionics_component_can_telemetry, state_error);
		return;
	}

	for (int i = 0; i < 4; i++) {
		msg->is_valid[i] = false;
	}
	msg->current_origin = telemetry_origin_invalid;
}

static bool isMultipacketValid(multipacket_message_t* msg) {
	for (int i = 0; i < msg->size_in_packets; i++)
		if (!msg->is_valid[i])
			return false;
	return true;
}

TELEMETRY_ALLOCATOR(can_telemetry_allocator, 1024);

void can_interface_init() {
	telemetry_allocator_init(&can_telemetry_allocator);
	COMPONENT_STATE_UPDATE(avionics_component_can_telemetry, state_ok);

	for (int i = 0; i < num_multipacket_messages; i++)
		resetMultipacketMessage(&multipacket_messages[i]);
}

static void handleFullPacket(uint16_t telemetry_id, telemetry_origin_t origin, uint8_t* data, uint8_t length) {
	telemetry_t* packet = telemetry_allocator_alloc(&can_telemetry_allocator, length);
	if (packet == NULL)
		return;
	memcpy(packet->payload, data, length);
	packet->header.id = telemetry_id;
	packet->header.length = length;
	packet->header.origin = origin;
	messaging_send(packet, 0);
}

static multipacket_message_t* getMultipacket(uint16_t telemetry_id) {
	for (int i = 0; i < num_multipacket_messages; i++) {
		uint16_t mask = ~(multipacket_messages[i].seqno_mask);
		if ((telemetry_id & mask) == multipacket_messages[i].base_id)
			return &multipacket_messages[i];
	}
	return NULL;
}

void can_recv(uint16_t can_msg_id, bool can_rtr, uint8_t *data, uint8_t datalen) {
	(void)can_rtr;
	uint16_t id = (can_msg_id >> 5) & 0x3F;
	uint8_t origin = can_msg_id & 0x1F;

	// We don't understand packets from other sources
	if (origin != telemetry_origin_avionics_gui && origin != telemetry_origin_m3imu)
		return;

	multipacket_message_t* multipacket = getMultipacket(id);
	if (multipacket == NULL) {
		if (datalen <= 8) {
			handleFullPacket(id, (telemetry_origin_t)origin, data, datalen);
			return;
		}
		else {
			COMPONENT_STATE_UPDATE(avionics_component_can_telemetry, state_error);
			return;
		}
	}

	if (multipacket->current_origin == telemetry_origin_invalid) {
		multipacket->current_origin = origin;
	} else if (multipacket->current_origin != origin) {
		COMPONENT_STATE_UPDATE(avionics_component_can_telemetry, state_error);
		return;
	}

	uint8_t seqno = id & multipacket->seqno_mask;

	uint8_t* ptr = (uint8_t*)&multipacket->data_buffer;
	ptr += seqno * 8;

	if (seqno*8 + datalen > multipacket->size_in_bytes) {
		COMPONENT_STATE_UPDATE(avionics_component_can_telemetry, state_error);
		return;
	}

	memcpy(ptr, data, datalen);

	multipacket->is_valid[seqno] = true;

	if (isMultipacketValid(multipacket)) {
		handleFullPacket(multipacket->base_id, multipacket->current_origin, multipacket->data_buffer, multipacket->size_in_bytes);
		resetMultipacketMessage(multipacket);
	}
}

// can_send is implemented by the platform

bool can_send_telemetry(const telemetry_t* packet, message_metadata_t metadata) {
    (void)metadata;
    if (packet->header.origin == local_config.origin) {
        if (packet->header.length <= 8) {
            // TODO: RTR?
            can_send((packet->header.id << 5) | local_config.origin, false, packet->payload, packet->header.length);
            return true;
        }


		multipacket_message_t* msg = getMultipacket(packet->header.id);
        if (msg == NULL) {
            COMPONENT_STATE_UPDATE(avionics_component_can_telemetry, state_error);
            return true;
        }

        uint8_t* ptr = packet->payload;
        int remaining = packet->header.length;
        int i = 0;
        do {
            can_send(((packet->header.id+i) << 5) | local_config.origin, false, ptr, remaining > 8 ? 8 : remaining);
            ptr += 8;
            i++;
            remaining -= 8;
        } while (remaining > 0);
    }
    return true;
}
