#ifndef STATE_ESTIMATE_CONFIG_H
#define STATE_ESTIMATE_CONFIG_H
#include "compilermacros.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// We use a right handed coordinate system with the y axis being up
typedef struct state_estimate_t {
	float orientation_q[4];
	float angular_velocity[3];
} state_estimate_t;

STATIC_ASSERT(sizeof(state_estimate_t) == 7 * 4, ms5611data_padded);

typedef struct state_estimate_calibration_t {
    // The length of time over which samples were taken
    float sample_time;
    float accel_bias[3];
    float gyro_bias[3];
    float mag_bias[3];
    float alt_offset;
} state_estimate_calibration_t;

typedef struct state_estimate_status_t {
     // The length of time over which samples were taken
    float sample_time;
    uint16_t number_prediction_steps;
    uint16_t number_update_steps;
} state_estimate_status_t;

STATIC_ASSERT(sizeof(state_estimate_calibration_t) == 44, state_estimate_calibration_padded);

#ifdef __cplusplus
}
#endif

#endif /* STATE_ESTIMATE_CONFIG_H */
