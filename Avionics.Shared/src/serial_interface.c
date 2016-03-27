#include "serial_interface.h"
#include <string.h>
#include "checksum.h"
/*

We use a byte stuffing algorithm based on PPP
The byte 0x7E is added to the beggining of every transmission
Whenever 0x7E or 0x7D appears in the data we write 0x7E followed by the byte xored with 0x20

i.e. 0x7E becomes 0x7D 0x5E and 0x7D becomes 0x7D 0x5D

*/


static bool write_bytes_to_buffer(serial_interface_t* serial_interface, const uint8_t* data, uint32_t length) {
    for (uint32_t i = 0; i < length; i++) {
        uint8_t byte = data[i];
        if (byte == 0x7E) {
            if (!serial_interface->stream_put(0x7D))
                return false;
            if (!serial_interface->stream_put(0x5E))
                return false;
        } else if (byte == 0x7D) {
            if (!serial_interface->stream_put(0x7D))
                return false;
            if (!serial_interface->stream_put(0x5D))
                return false;
        } else {
            if (!serial_interface->stream_put(byte))
                return false;
        }
    }
    return true;
}

bool serial_interface_send_packet(serial_interface_t* serial_interface, telemetry_t* packet) {
    if (!serial_interface->stream_put(0x7E))
        return false;
    
    if (!write_bytes_to_buffer(serial_interface, (uint8_t*)&packet->header, sizeof(packet->header)))
        return false;
        
    uint16_t header_crc = checksum_crc16((const uint8_t*)&packet->header, sizeof(packet->header));
    if (!write_bytes_to_buffer(serial_interface, (uint8_t*)&header_crc, sizeof(header_crc)))
        return false;
    

    if (!write_bytes_to_buffer(serial_interface, packet->payload, packet->header.length))
        return false;
    
    uint32_t payload_crc = checksum_crc32(packet->payload, packet->header.length);
    if (!write_bytes_to_buffer(serial_interface, (uint8_t*)&payload_crc, sizeof(payload_crc)))
        return false;

    if (serial_interface->stream_flush_write != NULL && !serial_interface->stream_flush_write())
        return false;

    return true;
}

static bool read_bytes_to_buffer(serial_interface_t* serial_interface, uint8_t* buffer, uint32_t length) {
    for (uint32_t i = 0; i < length; i++) {
        uint8_t byte = serial_interface->stream_get();
        if (byte == 0x7E)
            return false;
        if (byte == 0x7D) {
            uint8_t next_byte = serial_interface->stream_get();
            if (next_byte != 0x5E && next_byte != 0x5D)
                return false;
            buffer[i] = next_byte ^ 0x20;
        } else {
            buffer[i] = byte;
        }

    }
    return true;
}


static telemetry_t* serial_interface_read_frame(serial_interface_t* serial_interface) {
    // The start delimeter has already been read
    telemetry_header_t header;
    if (!read_bytes_to_buffer(serial_interface, (uint8_t*)&header, sizeof(header)))
        return NULL;
    uint16_t header_crc;
    if (!read_bytes_to_buffer(serial_interface, (uint8_t*)&header_crc, sizeof(header_crc)))
        return NULL;
    if (header_crc != checksum_crc16((const uint8_t*)&header, sizeof(header)))
        return NULL;

    telemetry_t* packet = telemetry_allocator_alloc(serial_interface->telemetry_allocator, header.length);
    if (packet == NULL)
        return NULL;

    read_bytes_to_buffer(serial_interface, packet->payload, header.length);

    uint32_t payload_crc = 0;
    read_bytes_to_buffer(serial_interface, (uint8_t*)&payload_crc, sizeof(payload_crc));
    
    if (payload_crc != checksum_crc32(packet->payload, header.length)) {
        telemetry_allocator_free(packet);
        return NULL;
    }

    memcpy(&packet->header, &header, sizeof(telemetry_header_t));
    return packet;
}

telemetry_t* serial_interface_next_packet(serial_interface_t* serial_interface) {
    while (true) {
        while (serial_interface->stream_get() != 0x7E) {}
        struct telemetry_t* ret;
        if ((ret = serial_interface_read_frame(serial_interface)) != NULL)
            return ret;
    }
}