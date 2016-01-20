

#ifndef BAD_THING_HANDLER_H
#define BAD_THING_HANDLER_H

#include <stdbool.h>

typedef enum {
	ERROR_GYRO_MAGNO_ACCEL = 0,
	ERROR_RADIO,
	ERROR_GPS,
	ERROR_ALTIM,
	ERROR_SD_CARD,
	ERROR_CONFIG,
	ERROR_MAX,
} bthandler_error_t;

extern volatile bool error_states[ERROR_MAX];

/* Set a specific error.
* If `set` is true then the error is occurring.
*/
void bthandler_set_error(bthandler_error_t err, bool set);
msg_t bthandler_thread(void* arg);

#endif /* BAD_THING_HANDLER_H*/