#include <component_state.h>
#include <messaging.h>
#include <config/telemetry_packets.h>
#include "ublox.h"
#include "ublox_ubx.h"
#include "ch.h"
#include "hal.h"
#include "spalaxconf.h"

#if HAS_UBLOX

// We use UBX_CLASS_ACK as a sentinel value, as an ACK can never be expected for an ACK
static uint8_t pending_ack_class = UBX_CLASS_ACK;
static uint8_t pending_ack_id = 0;

bool ublox_next_packet();

static void compute_checksum(uint8_t* buf, uint8_t* ck_a_out, uint8_t* ck_b_out) {
    ubx_header_t* header = (ubx_header_t*)buf;

    uint8_t ck_a = 0x00, ck_b = 0x00;
    for(int i=2; i<header->length+sizeof(ubx_header_t); i++) {
        ck_a += buf[i];
        ck_b += ck_a;
    }
    *ck_a_out = ck_a;
    *ck_b_out = ck_b;
}

static void frame_message(uint8_t *buf) {
    ubx_header_t* header = (ubx_header_t*)buf;
    header->preamble1 = UBX_PREAMBLE1;
    header->preamble2 = UBX_PREAMBLE2;

    compute_checksum(buf, &buf[header->length + sizeof(ubx_header_t)],
                     &buf[header->length + + sizeof(ubx_header_t) + 1]);
}

static bool check_message(uint8_t *buf) {
    ubx_header_t* header = (ubx_header_t*)buf;
    if(header->preamble1 != UBX_PREAMBLE1 && header->preamble2 != UBX_PREAMBLE2) {
        return false;
    }

    uint8_t ck_a, ck_b;
    compute_checksum(buf, &ck_a, &ck_b);
    return ck_a == buf[header->length + sizeof(ubx_header_t)] &&
           ck_b == buf[header->length + + sizeof(ubx_header_t) + 1];
}

void send_message(uint8_t* buf) {
    ubx_header_t* header = (ubx_header_t*)buf;
    frame_message(buf);
    sdWrite(&SD1, buf, header->length + sizeof(ubx_header_t) + 2);
}

bool send_message_ack(uint8_t* buf) {
    bool last_ret = true;
    ubx_header_t* header = (ubx_header_t*)buf;
    pending_ack_class = header->msg_class;
    pending_ack_id = header->msg_id;

    send_message(buf);

    while(pending_ack_class != UBX_CLASS_ACK) {
        last_ret = ublox_next_packet();
    }
    return last_ret;
}

bool process_ack_message(uint8_t* buf) {
    ubx_ack_t* ack = (ubx_ack_t*) buf;
    if (pending_ack_class == UBX_CLASS_ACK)
        return true;
    if (ack->ack_msg_class != pending_ack_class || ack->ack_msg_id != pending_ack_id)
        return true;
    pending_ack_class = UBX_CLASS_ACK;
    return ack->header.msg_id == UBX_ACK_ACK;
}

MESSAGING_PRODUCER(messaging_producer, ts_ublox_nav, sizeof(ublox_nav_t), 10)

bool process_nav_message(uint8_t* buf) {
    ubx_header_t* header = (ubx_header_t*)buf;
    if (header->msg_id != UBX_NAV_PVT)
        return false;

    ubx_nav_pvt_t* msg = (ubx_nav_pvt_t*)buf;

    ublox_nav_t data;
    data.year = msg->year;
    data.month = msg->month;
    data.day = msg->day;
    data.hour = msg->hour;
    data.minute = msg->minute;
    data.second = msg->second;
    data.valid = msg->valid;
    data.t_acc = msg->t_acc;
    data.nano = msg->nano;
    data.fix_type = msg->fix_type;
    data.flags = msg->flags;
    data.num_sv = msg->num_sv;
    data.lon = msg->lon;
    data.lat = msg->lat;
    data.height = msg->height;
    data.h_msl = msg->h_msl;
    data.h_acc = msg->h_acc;
    data.v_acc = msg->v_acc;
    data.velN = msg->velN;
    data.velE = msg->velE;
    data.velD = msg->velD;
    data.s_acc = msg->s_acc;
    data.p_dop = msg->p_dop;

    messaging_producer_send(&messaging_producer, message_flags_send_over_can, (const uint8_t*)&data);

    return true;
}

bool process_message(uint8_t* buf) {
    ubx_header_t* header = (ubx_header_t*)buf;
    switch (header->msg_class) {
        case UBX_CLASS_ACK:
            return process_ack_message(buf);
        case UBX_CLASS_NAV:
            return process_nav_message(buf);
        default:
            return false;
    }
}

bool ublox_next_packet() {
    static uint8_t read_buffer[128];
    static ubx_header_t* header = (ubx_header_t*)read_buffer;

    while(true) {
        if ((read_buffer[0] = (uint8_t)sdGet(&SD1)) != UBX_PREAMBLE1)
            continue;
        if ((read_buffer[1] = (uint8_t)sdGet(&SD1)) != UBX_PREAMBLE2)
            continue;

        sdRead(&SD1, read_buffer+2, sizeof(ubx_header_t) - 2);
        if (header->length >= sizeof(read_buffer) - sizeof(ubx_header_t) - 2) {
            continue;
        }

        sdRead(&SD1, read_buffer+sizeof(ubx_header_t), header->length + 2);

        if (check_message(read_buffer)) {
            return process_message(read_buffer);
        }
    }
}

static bool ublox_init(void) {
    messaging_producer_init(&messaging_producer);

    ubx_cfg_msg_t cfg_msg = {.header = UBX_CFG_MSG_HEADER};

    // Disable NMEA
    for (uint8_t i = 0x00; i <= 0x0A; i++) {
        cfg_msg.cfg_msg_class = NMEA_CLASS;
        cfg_msg.cfg_msg_id = i;
        cfg_msg.rate = 0;
        if (!send_message_ack((uint8_t*)&cfg_msg))
            return false;
    }

    cfg_msg.cfg_msg_class = UBX_CLASS_NAV;
    cfg_msg.cfg_msg_id = UBX_NAV_PVT;
    cfg_msg.rate = 1;
    if (!send_message_ack((uint8_t*)&cfg_msg))
        return false;

    ubx_cfg_rate_t cfg_rate = {.header = UBX_CFG_RATE_HEADER, .meas_rate = 100, .nav_rate = 1, .time_ref = 0};
    if (!send_message_ack((uint8_t*)&cfg_rate))
        return false;

    ubx_cfg_nav5_t cfg_nav5 = {
            .header = UBX_CFG_NAV5_HEADER,
            .mask = 1, // Set the parameter mask to include below parameters
            .dyn_model = 8, // Set the model to airborne <4g acceleration
    };
    if (!send_message_ack((uint8_t*)&cfg_nav5))
        return false;


    return false;
}

void ublox_thread(void *arg) {
    chRegSetThreadName("UBLOX");

    COMPONENT_STATE_UPDATE(avionics_component_ublox, state_initializing);

    SerialConfig uartCfg = {
            9600, // bit rate
    };

    sdStart(&SD1, &uartCfg);

    COMPONENT_STATE_UPDATE(avionics_component_ublox, state_initializing);

    ublox_init();

    COMPONENT_STATE_UPDATE(avionics_component_ublox, state_ok);

    while (true) {
        ublox_next_packet();
        chThdYield();
    }
}


#else

#include "ublox.h"

void ublox_thread(void *arg) {
    COMPONENT_STATE_UPDATE(avionics_component_ublox, state_error);
}

#endif


