#ifndef STATE_ESTIMATE_CONFIG_H
#define STATE_ESTIMATE_CONFIG_H
#include "compilermacros.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// We use a right handed coordinate system with the y axis being up
typedef struct state_estimate_t {
    float pos[3]; // Position in relation to start
    float vel[3];
    float accel[3];

    float angular_velocity[3];
    float orientation_q[4];
} state_estimate_t;

STATIC_ASSERT(sizeof(state_estimate_t) == 16 * 4, ms5611data_padded);

typedef struct state_estimate_calibration_t {
    float sample_time; // The length of time over which samples were taken

    // The number of samples taken
    uint16_t accel_samples;
    uint16_t gyro_samples;
    uint16_t mag_samples;
    uint16_t alt_samples;

    // The average readings obtained
    float accel_bias[3];
    float gyro_bias[3];
    float mag_bias[3];
    float alt_offset;
} state_estimate_calibration_t;

STATIC_ASSERT(sizeof(state_estimate_calibration_t) == 52, state_estimate_calibration_padded);


#ifdef __cplusplus
}
#endif

#endif /* STATE_ESTIMATE_CONFIG_H */
