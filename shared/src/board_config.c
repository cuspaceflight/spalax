#include "board_config.h"
#include <string.h>
#include <avionics_config.h>

int num_board_configs = 2;
board_config_t board_configs[2] = {
    { // Dart
        .board_id = {3407947, 859066637, 858994999},
        .mpu9250_magno_sf = {3780, 2610, 1956},
        .mpu9250_magno_bias = {-10, 383, -257},
        .has_adis = true,
        .accel_reference = {0.278013796, 0.344436288, 9.90290546},
        .magno_reference =   {0.209259585, -0.125725865, -0.54462719}
    },
    { // Booster
        .board_id = {3801163, 859066637, 858994999},
        .mpu9250_magno_sf = {3868,3683,3656},
        .mpu9250_magno_bias = {54,266,-37},
        .has_adis = false,
        .accel_reference = {0.404072106, 0.344563782, 9.53682613},
        .magno_reference = {0.293875277, -0.15511699, -0.872909367}
    }

};


void getBoardID(uint32_t data[3]) {
	// TODO: Fix for desktop
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
