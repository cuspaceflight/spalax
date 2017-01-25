#include "board_config.h"
#include <string.h>


board_config_t board_configs[BoardConfigMax] = {
        {
                .name = BoardConfigM3Dart,
                .board_id = {3407947, 859066637, 858994999},
                .has_adis = false,
                .has_gps = false,
                .has_mpu9250 = true,
                .has_ms5611 = true,
        },
        {
                .name = BoardConfigM3Booster,
                .board_id = {3801163, 859066637, 858994999},
                .has_adis = false,
                .has_gps = false,
                .has_mpu9250 = true,
                .has_ms5611 = true,
        },
        {
                .name = BoardConfigSpalax,
                .board_id = {3407919, 875778316, 808991032},
                .has_adis = false,
                .has_gps = false,
                .has_mpu9250 = true,
                .has_ms5611 = true,

                .mpu9250_magno_transform = {3.85244896e-03f,   2.37093138e-06f,  -3.59720029e-06f,
                                            2.37093138e-06f,   3.85718695e-03f,   1.57757849e-05f,
                                            -3.59720029e-06f,  1.57757849e-05f,   3.78287880e-03f},
                .mpu9250_magno_offset = {-124.68356118f, -249.45515055f,  383.40074598f},

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
    return NULL;
}


