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
                .name = BoardConfigM3Dart2,
                .board_id = {3342426, 859197701, 959525687},
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

                .mpu9250_magno_transform = {3.61694852e-03f, 1.20641814e-06f, 1.36351719e-06f,
                                            1.20641814e-06f, 3.64952393e-03f, 7.28715828e-05f,
                                            1.36351719e-06f, 7.28715828e-05f, 3.45339666e-03f},


                .mpu9250_magno_offset = {-245.80496207f, -133.42258141f, -355.85977773f},

                .mpu9250_accel_transform = {
                        4.91349087e-04f, 4.30897731e-06f, -7.38765621e-07f,
                        4.30897731e-06f, 4.98870383e-04f, -2.67767187e-06f,
                        -7.38765621e-07f, -2.67767187e-06f, 4.83404457e-04f,
                },
                .mpu9250_accel_offset = {-20.62997267f, 19.17199868f, 60.14190283f},

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
                .run_state_estimators = true,

                .mpu9250_magno_transform = {  0.00356189f,   5.5954e-05f,  0.000142585f,
                                              -7.67756e-05f,   0.00353788f,   0.00021073f,
                                              -0.00018975f, -0.000203796f,   0.00339035f},

                .mpu9250_magno_offset = {-161.21354731f, -71.48453403f, -396.80383599f},

                .mpu9250_accel_transform = {
                        4.96157431e-04f, 7.87359021e-06f, -1.98588249e-06f,
                        7.87359021e-06f, 4.88907462e-04f, -6.34380503e-07f,
                        -1.98588249e-06f, -6.34380503e-07f, 4.81342479e-04f
                },

                .mpu9250_accel_offset = {33.26432818f,  36.48421574f, 47.63982994f},

                .mpu9250_gyro_sf = 500.0f * 0.01745329251f / 32767.0f,

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


