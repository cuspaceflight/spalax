#ifndef STATE_ESTIMATE_H
#define STATE_ESTIMATE_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// We use a right handed coordinate system with the y axis being up
typedef struct state_estimate_t {
    float pos[3];
    float vel[3];
    float accel[3];

    float angular_velocity[3];

    float orientation_q[4];
} state_estimate_t;

void reset_state_estimate(state_estimate_t* estimate);

void print_state_estimate(const state_estimate_t* estimate);

//void state_estimate_compute_next(state_estimate_t* next_estimate, float dt);

#ifdef __cplusplus
}
#endif

#endif /* STATE_ESTIMATE_H */
