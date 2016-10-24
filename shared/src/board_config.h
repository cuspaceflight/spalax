#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t board_id[3];
    uint16_t mpu9250_magno_bias[3];
    uint16_t mpu9250_magno_sf[3];
    float accel_reference[3];
    float magno_reference[3];
    bool has_adis;
} board_config_t;

board_config_t* getBoardConfig(void);

void getBoardID(uint32_t data[3]);

#endif
