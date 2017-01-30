#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "compilermacros.h"

typedef enum {
    BoardConfigM3Dart,
    BoardConfigM3Booster,
    BoardConfigSpalax,
    BoardConfigSpalaxBrokenSD,
    BoardConfigMax
} BoardConfig;

typedef struct {
    uint32_t board_id[3];
    BoardConfig name;

    bool has_ms5611;
    bool has_mpu9250;
    bool has_adis;
    bool has_gps;
    bool has_sdcard;

    bool run_state_estimators;

    // Row Major 3x3 Matrix
    float mpu9250_magno_transform[9];
    float mpu9250_magno_offset[3];

    // Row Major 3x3 Matrix
    float mpu9250_accel_transform[9];
    float mpu9250_accel_offset[3];

    float mpu9250_gyro_sf;
} board_config_t;


board_config_t *getBoardConfig(void);

#ifdef MESSAGING_OS_STD

void setBoardConfig(BoardConfig config);

#endif

#ifdef __cplusplus
}
#endif

#endif
