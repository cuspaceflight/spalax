#include "messaging.h"
#include "hal.h"
#include "chtypes.h"
#include <string.h>

typedef struct messaging_delegate_t {
    messaging_delegate_func delegate_func;

    // Each delegate has its own mailbox and memory pool
    // This is so that if a delegate is not retrieving messages (e.g it has crashed)
    // It will not impact any other delegates, its mailbox/pool will fill up
    // And stop receiving further messages
    Mailbox mailbox;
    MemoryPool memory_pool;
    // I don't think MemoryPool is thread safe so we protect it with a mutex
    Mutex memory_pool_mutex;

    struct messaging_delegate_t* next;
} messaging_delegate_t;

#define MAX_NUM_DELEGATES 10

// Maximum number of packets waiting for delegates - delegates 'reserve' space
// In the relevant buffers when they register and so if this number is
// insufficient an error will be thrown on startup as oppossed at some later date
#define DELEGATE_BUFFER_SIZE 3072


// The number in the messaging_delegates_pool when registering the next delegate
static volatile int cur_delegate_pool_index = 0;
// The amount, in units of telemetry_t, of space reserved in the delegate buffers
static volatile int delegate_pool_reserved_buffer_index = 0;


static volatile msg_t delegate_mailbox_buffer[DELEGATE_BUFFER_SIZE]
                     __attribute__((section(".ccm")));

static volatile char delegate_mempool_buffer[DELEGATE_BUFFER_SIZE * sizeof(telemetry_t)]
                    __attribute__((aligned(sizeof(stkalign_t))))
                    __attribute__((section(".ccm")));

// Delegates are allocated from here. This is so that the messaging component
// 'owns' them (i.e. they won't get cleaned up for any reason)
// It also hides the implementation details from any user of this component
static messaging_delegate_t messaging_delegates_pool[MAX_NUM_DELEGATES];

// Used to make delegate registration synchronous
// This protects cur_delegate_pool_index and delegate_pool_reserved_buffer_index
// and the allocation of sections of
// delegate_mailbox_buffer and delegate_mempool_buffer
// This could be made more fine-grained but I see no major reason to
static Mutex messaging_register_mutex;

// Maps each telemetry source to a linked list of messaging delegates
static messaging_delegate_t* messaging_delegates[16];

static Mutex messaging_delegates_locks[16];

void messaging_init(void) {
    chMtxInit(&messaging_register_mutex);
    for (int i = 0; i < 16; i++) {
        messaging_delegates[i] = 0;
        chMtxInit(&messaging_delegates_locks[i]);
    }

    cur_delegate_pool_index = 0;
}

bool messaging_register_delegate(message_delegate_id* delegate_id, messaging_delegate_func func, telemetry_source_t source, uint32_t buffer_size) {
    *delegate_id = 255;
    chMtxLock(&messaging_register_mutex);

    if (cur_delegate_pool_index >= MAX_NUM_DELEGATES || source >= 16) {
        chMtxUnlock(); // messaging_register_mutex
        return false; // TODO: Log some sort of error
    }


    // We first make sure we have sufficient space in the delegate buffers
    if (delegate_pool_reserved_buffer_index + buffer_size > DELEGATE_BUFFER_SIZE) {
        chMtxUnlock(); // messaging_register_mutex
        return false; // TODO: Log some sort of error
    }


    messaging_delegate_t* delegate = &(messaging_delegates_pool[cur_delegate_pool_index]);
    *delegate_id = cur_delegate_pool_index;
    cur_delegate_pool_index++;


    delegate->delegate_func = func;

    chMtxInit(&(delegate->memory_pool_mutex));
    chMBInit(&(delegate->mailbox), (msg_t*)&(delegate_mailbox_buffer[delegate_pool_reserved_buffer_index]), buffer_size);
    chPoolInit(&(delegate->memory_pool), sizeof(telemetry_t), NULL);
    chPoolLoadArray(&(delegate->memory_pool), (void*)&(delegate_mempool_buffer[delegate_pool_reserved_buffer_index*sizeof(telemetry_t)]), buffer_size);

    delegate_pool_reserved_buffer_index += buffer_size;

    chMtxUnlock(); // messaging_register_mutex

    chMtxLock(&messaging_delegates_locks[source]);
    delegate->next = messaging_delegates[source];
    messaging_delegates[source] = delegate;
    chMtxUnlock(); // messaging_delegates_locks[source]


    return true;
}

static void messaging_send_message_to_delegate(telemetry_t* message, messaging_delegate_t* delegate) {
    // Create copy of message in delegates memory pool
    // Insert into delegate's Mailbox

    void* msg;
    chMtxLock(&delegate->memory_pool_mutex);
    msg = chPoolAlloc(&delegate->memory_pool);
    chMtxUnlock();

    if (msg == 0)
        return; // TODO: Log an error
    memcpy(msg, (void*)message, sizeof(telemetry_t));

    msg_t retval = chMBPost(&delegate->mailbox, (intptr_t)msg, TIME_IMMEDIATE);
    if (retval != RDY_OK) {
        chMtxLock(&delegate->memory_pool_mutex);
        chPoolFree(&delegate->memory_pool, msg);
        chMtxUnlock();
        return;
    }
}

static void messaging_send_message_to_source(telemetry_t* message, telemetry_source_t source) {
    chMtxLock(&messaging_delegates_locks[source]);
    messaging_delegate_t* current_delegate = messaging_delegates[source];
    while (current_delegate != 0) {
        messaging_send_message_to_delegate(message, current_delegate);
        current_delegate = current_delegate->next;
    }
    chMtxUnlock();
}

void messaging_send_message(telemetry_t* message) {
    telemetry_source_t source = telemetry_get_source(message);
    messaging_send_message_to_source(message, source);
    messaging_send_message_to_source(message, telemetry_source_wildcard);
}

bool messaging_process_message(message_delegate_id* delegate_id, bool blocking, bool silent) {
    if (*delegate_id >= MAX_NUM_DELEGATES)
        return false;

    intptr_t data_msg;
    messaging_delegate_t* delegate = &(messaging_delegates_pool[*delegate_id]);
    msg_t mailbox_ret = chMBFetch(&(delegate->mailbox), (msg_t*)&data_msg, blocking ? TIME_INFINITE : TIME_IMMEDIATE);
    if (mailbox_ret != RDY_OK || data_msg == 0)
        return false;

    if (!silent) {
        delegate->delegate_func((telemetry_t*)data_msg);
    }

    chMtxLock(&delegate->memory_pool_mutex);
    chPoolFree(&delegate->memory_pool, (void*)data_msg);
    chMtxUnlock();
    return true;
}
