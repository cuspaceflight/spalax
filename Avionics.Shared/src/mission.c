#include <mission.h>
#include <logging.h>

void print_state_transition(int from_state, int to_state) {
	static const char* lookup[NUM_STATES] = { "STATE_PAD", "STATE_IGNITION", "STATE_POWERED_ASCENT", "STATE_FREE_ASCENT",
		"STATE_APOGEE", "STATE_DROGUE_DESCENT", "STATE_RELEASE_MAIN",
		"STATE_MAIN_DESCENT", "STATE_LAND", "STATE_LANDED" };
	if (from_state < 0 || from_state > NUM_STATES || to_state < 0 || to_state > NUM_STATES) {
		PRINT("\nIllegal State!\n\n");
		return;
	}

	PRINT("\nState transition from %s to %s\n\n", lookup[from_state], lookup[to_state]);
}