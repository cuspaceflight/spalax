#include "board_config.h"
#include <string.h>

int num_board_configs = 2;
board_config_t board_configs[2] = {
    { // Dart
        .board_id = {3407947, 859066637, 858994999},
        .mpu9250_magno_sf = {3780, 2610, 1956},
        .mpu9250_magno_bias = {-10, 383, -257},
        .has_adis = true
    },
    { // Booster
        .board_id = {3801163, 859066637, 858994999},
        .mpu9250_magno_sf = {3868,3683,3656},
        .mpu9250_magno_bias = {54,266,-37},
        .has_adis = false
    }

};


void getBoardID(uint32_t data[3]) {
    memcpy(data, (uint32_t*)0x1FFF7A10, 3 * sizeof(uint32_t));
}

board_config_t* getBoardConfig(void) {
    uint32_t* board_id = (uint32_t*)0x1FFF7A10;
    for (int i = 0; i < num_board_configs; i++) {
        board_config_t* config = &board_configs[i];
        bool is_equal = true;
        for (int j = 0; j < 3; j++) {
            if (config->board_id[j] != board_id[j]) {
                is_equal = false;
                break;
            }
        }
        if (is_equal)
            return config;
    }
    return NULL;
}
