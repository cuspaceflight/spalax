#ifndef MESSAGING_H
#define MESSAGING_H
#include "telemetry.h"
#include <stdbool.h>
#include "chtypes.h"

typedef uint8_t message_delegate_id;
typedef void(*messaging_delegate_func)(telemetry_t*);

void messaging_init(void);

// Specify a function to invoke with messages
// A source to get messages for
// And a buffer size in units of telemetry_t
// Once this buffer fills up no further messages will be enqueued for this delegate
// Until some messages are removed
// If succesful will return true and set message_delegate_id to a valid value
// Otherwise will return false
bool messaging_register_delegate(message_delegate_id* delegate_id, messaging_delegate_func func, telemetry_source_t source, uint32_t buffer_size);

void messaging_send_message(telemetry_t* message);

// Get the next message
// Returns true if there was a message
// Will wait for the specified timeout
// If silent is true will not invoke the delegate function
bool messaging_process_message(message_delegate_id* delegate, systime_t timeout, bool silent);

#endif /* MESSAGING_H */
