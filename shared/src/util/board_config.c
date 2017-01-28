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
        },
        {
                .name = BoardConfigM3Booster,
                .board_id = {3801163, 859066637, 858994999},
                .has_adis = false,
                .has_gps = false,
                .has_mpu9250 = true,
                .has_ms5611 = true,
                .has_sdcard = false,
        },
        {
                .name = BoardConfigSpalax,
                .board_id = {3407919, 875778316, 808991032},
                .has_adis = false,
                .has_gps = false,
                .has_mpu9250 = true,
                .has_ms5611 = true,
                .has_sdcard = true,

                .mpu9250_magno_transform = {3.83654358e-03f,  -3.16423517e-05f,   1.81636820e-05f,
                                            -3.16423517e-05f,   3.84013596e-03f,   4.12319935e-05f,
                                            1.81636820e-05f,   4.12319935e-05f,  3.70062710e-03f},
                .mpu9250_magno_offset = {-135.21782686f, -261.42029277f,  383.75197833f},

                .mpu9250_accel_transform = {4.50911663e-04f,  -7.19296094e-05f,   2.31455143e-05f,
                                            -7.19296094e-05f,  4.70336973e-04f,   1.79558214e-05f,
                                            2.31455143e-05f,   1.79558214e-05f,   4.90503092e-04f},
                .mpu9250_accel_offset = {-47.8512882f,  -11.24502436f,   5.30242719f},

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
        }
};

#if defined(MESSAGING_OS_STD)
static uint32_t board_id[3] = {0,0,0};

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
    static board_config_t* cached = NULL;
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


