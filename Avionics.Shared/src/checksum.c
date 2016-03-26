#include "checksum.h"

// CRC32 implementation from http://www.hackersdelight.org/hdcodetxt/crc.c.txt

static uint32_t crc32_lookup[256];

static void gen_crc32_lookup(void) {
    uint32_t crc, mask;
	for (uint32_t byte = 0; byte <= 255; byte++) {
		crc = byte;
		for (int j = 7; j >= 0; j--) { // Do eight times.
			mask = -(crc & 1);
			crc = (crc >> 1) ^ (0xEDB88320 & mask);
		}
		crc32_lookup[byte] = crc;
	}
}

void checksum_init(void) {
    gen_crc32_lookup();
}

uint32_t checksum_crc32(const uint8_t* buffer, uint32_t length) {
	uint32_t crc = 0xFFFFFFFF;
	for (uint32_t i = 0; i < length; i++) {
        uint8_t byte = buffer[i];
		crc = (crc >> 8) ^ crc32_lookup[(crc ^ byte) & 0xFF];
	}
	return ~crc;
}

// CRC16 implementation from https://github.com/cuspaceflight/m2-electronics/blob/master/m2serial/m2serial.c
uint16_t checksum_crc16(const uint8_t* buffer, uint32_t length) {
    size_t i, j;
    uint16_t crc = 0xFFFF;
    for (i = 0; i<length; i++) {
        crc ^= (uint16_t)buffer[i] << 8;
        for (j = 0; j<8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            }
            else {
                crc <<= 1;
            }
        }
    }
    return ~crc;
}
