#ifndef CHECKSUM_H
#define CHECKSUM_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void checksum_init(void);


uint16_t checksum_crc16(const uint8_t* buffer, uint32_t length);

uint32_t checksum_crc32(const uint8_t* buffer, uint32_t length);


#ifdef __cplusplus
}
#endif

#endif /* CHECKSUM_H */
