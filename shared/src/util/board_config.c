#include "board_config.h"
#include <string.h>
#include <component_state.h>


board_config_t board_configs[BoardConfigMax] = {
        {
                .name = BoardConfigM3Dart,
                .board_id = {3407947, 859066637, 858994999},
                .has_adis = false,
                .has_gps = false,
                .has_mpu9250 = true,
                .has_ms5611 = true,
                .has_sdcard = false,
                .run_state_estimators = false,
        },
        {
                .name = BoardConfigM3Booster,
                .board_id = {3801163, 859066637, 858994999},
                .has_adis = false,
                .has_gps = false,
                .has_mpu9250 = true,
                .has_ms5611 = true,
                .has_sdcard = false,
                .run_state_estimators = false,
        },
        {
                .name = BoardConfigSpalax,
                .board_id = {3407919, 875778316, 808991032},
                .has_adis = false,
                .has_gps = false,
                .has_mpu9250 = true,
                .has_ms5611 = true,
                .has_sdcard = false,
                .run_state_estimators = false,


                .mpu9250_magno_transform = {  0.00376512f,  -0.00029081f, -6.19474e-05f,
                                              0.000233028f,   0.00373034f,  0.000145561f,
                                              5.97238e-05f, -0.000127953f,   0.00362172f},
                // Pre alignment calibration
//                .mpu9250_magno_transform = {3.77266559e-03f, -3.10432818e-05f, 2.35129433e-06f,
//                                            -3.10432818e-05f, 3.74368523e-03f, 1.32543291e-05f,
//                                            2.35129433e-06f, 1.32543291e-05f, 3.62513894e-03f},

                .mpu9250_magno_offset = { -251.84631888f, -134.48598684f, -361.30154678f },

                .mpu9250_accel_transform = {
                    4.91349087e-04f, 4.30897731e-06f, -7.38765621e-07f,
                            4.30897731e-06f, 4.98870383e-04f, -2.67767187e-06f,
                            -7.38765621e-07f, -2.67767187e-06f, 4.83404457e-04f,
                },
                .mpu9250_accel_offset = { -20.62997267f, 19.17199868f, 60.14190283f },

                .mpu9250_gyro_sf = 500.0f * 0.01745329251f / 32767.0f,
        },
        {
                .name = BoardConfigSpalaxBrokenSD,
                .board_id = {3670059, 875778316, 808991032},
                .has_adis = false,
                .has_gps = false,
                .has_mpu9250 = true,
                .has_ms5611 = true,
                .has_sdcard = false,
                .run_state_estimators = false,
        }
};

#if defined(MESSAGING_OS_STD)
static uint32_t board_id[3] = {0, 0, 0};

void setBoardConfig(BoardConfig config_name) {
    for (int i = 0; i < BoardConfigMax; i++) {
        board_config_t *config = &board_configs[i];
        if (config->name == config_name) {
            memcpy(board_id, config->board_id, 3 * sizeof(uint32_t));
            return;
        }
    }
}

static void getBoardID(uint32_t data[3]) {
    memcpy(data, board_id, 3 * sizeof(uint32_t));
}

#elif defined(MESSAGING_OS_CHIBIOS)

static void getBoardID(uint32_t data[3]) {
    memcpy(data, (uint32_t *) 0x1FFF7A10, 3 * sizeof(uint32_t));
}

#endif

board_config_t *getBoardConfig(void) {
    static board_config_t *cached = NULL;
    if (cached)
        return cached;

    uint32_t id[3];
    getBoardID(id);
    for (int i = 0; i < BoardConfigMax; i++) {
        board_config_t *config = &board_configs[i];
        bool is_equal = true;
        for (int j = 0; j < 3; j++) {
            if (config->board_id[j] != id[j]) {
                is_equal = false;
                break;
            }
        }
        if (is_equal) {
            cached = config;
            return cached;
        }
    }
    COMPONENT_STATE_UPDATE(avionics_component_state_board_config, state_error);

    return NULL;
}


