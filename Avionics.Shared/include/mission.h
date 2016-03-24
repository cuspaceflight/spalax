#ifndef MISSION_H
#define MISSION_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	STATE_PAD = 0, STATE_IGNITION, STATE_POWERED_ASCENT, STATE_FREE_ASCENT,
	STATE_APOGEE, STATE_DROGUE_DESCENT, STATE_RELEASE_MAIN,
	STATE_MAIN_DESCENT, STATE_LAND, STATE_LANDED, NUM_STATES
} state_t;

void print_state_transition(int from_state, int to_state);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* MISSION_H */
