#include <mutex>
#include <TQueue.h>

extern "C" {
#include "messaging.h"
}

#define MAX_NUM_DELEGATES 10

struct messaging_delegate_t {
    messaging_delegate_func delegate_func;

    TQueue<telemetry_t> mailbox;
    messaging_delegate_t* next;
};

// The number in the messaging_delegates_pool when registering the next delegate
static volatile int cur_delegate_pool_index = 0;

// Delegates are allocated from here. This is so that the messaging component
// 'owns' them (i.e. they won't get cleaned up for any reason)
// It also hides the implementation details from any user of this component
static messaging_delegate_t messaging_delegates_pool[MAX_NUM_DELEGATES];

// Used to make delegate registration synchronous
// This protects cur_delegate_pool_index
// This could be made more fine-grained but I see no major reason to
static std::mutex messaging_register_mutex;

// Maps each telemetry source to a linked list of messaging delegates
static messaging_delegate_t* messaging_delegates[16];

static std::mutex messaging_delegates_locks[16];

extern "C" void messaging_init(void) {
    for (int i = 0; i < 16; i++) {
        messaging_delegates[i] = nullptr; 
    }
}

extern "C" bool messaging_register_delegate(message_delegate_id* delegate_id, messaging_delegate_func func, telemetry_source_t source, uint32_t buffer_size) {
    *delegate_id = 255;
    messaging_delegate_t* delegate;
    {
        std::lock_guard<std::mutex> lock(messaging_register_mutex);

        if (cur_delegate_pool_index >= MAX_NUM_DELEGATES || source >= 16) {
            return false; // TODO: Log some sort of error
        }


        delegate = &messaging_delegates_pool[cur_delegate_pool_index];
        *delegate_id = cur_delegate_pool_index;
        cur_delegate_pool_index++;
    }
    

    delegate->delegate_func = func;

    {
        std::lock_guard<std::mutex> source_lock(messaging_delegates_locks[source]);
        delegate->next = messaging_delegates[source];
        messaging_delegates[source] = delegate;
    }
    return true;
}

static void messaging_send_message_to_delegate(telemetry_t* message, messaging_delegate_t* delegate) {
    delegate->mailbox.enqueue(*message);
}

static void messaging_send_message_to_source(telemetry_t* message, telemetry_source_t source) {
    std::lock_guard<std::mutex> lock(messaging_delegates_locks[source]);
    messaging_delegate_t* current_delegate = messaging_delegates[source];
    while (current_delegate != nullptr) {
        messaging_send_message_to_delegate(message, current_delegate);
        current_delegate = current_delegate->next;
    }
}

extern "C" void messaging_send_message(telemetry_t* message) {
    messaging_send_message_to_source(message, (telemetry_source_t)message->source);
    messaging_send_message_to_source(message, telemetry_source_wildcard);
}


extern "C" bool messaging_process_message(message_delegate_id* delegate_id, bool blocking, bool silent) {
    if (*delegate_id >= MAX_NUM_DELEGATES)
        return false;

    auto delegate = &messaging_delegates_pool[*delegate_id];
    if (!blocking && delegate->mailbox.isEmpty())
        return false;

    auto msg = delegate->mailbox.dequeue();

    if (!silent) {
        delegate->delegate_func(&msg);
    }

    return true;
}