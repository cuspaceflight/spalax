#ifndef UBLOX_UBX_H
#define UBLOX_UBX_H

#include "compilermacros.h"

enum ubx_msg_class {
    UBX_CLASS_NAV = 0x01,
    UBX_CLASS_RXM = 0x02,
    UBX_CLASS_INF = 0x04,
    UBX_CLASS_ACK = 0x05,
    UBX_CLASS_CFG = 0x06,
    UBX_CLASS_UPD = 0x09,
    UBX_CLASS_MON = 0x0A,
    UBX_CLASS_AID = 0x0B,
    UBX_CLASS_TIM = 0x0D,
    UBX_CLASS_MGA = 0x13,
    UBX_CLASS_LOG = 0x21,
    NMEA_CLASS = 0xF0,
};

// Selection of message ids
enum ubx_msg_id {
    UBX_ACK_ACK = 0x01,
    UBX_ACK_NACK = 0x00,

    UBX_CFG_MSG = 0x01,
    UBX_CFG_RATE = 0x08,
    UBX_CFG_NAV5 = 0x24,

    UBX_NAV_PVT = 0x07,

};

enum ubx_nav_fix_type {
    FIX_NONE = 0,
    FIX_DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME = 5
};

enum ubx_gnss_identifier {
    GNSS_GPS     = 0x00,
    GNSS_SBAS    = 0x01,
    GNSS_GALILEO = 0x02,
    GNSS_BEIDOU  = 0x03,
    GNSS_IMES    = 0x04,
    GNSS_QZSS    = 0x05,
    GNSS_GLONASS = 0x06
};

enum ubx_hardware_version {
    ANTARIS = 0,
    UBLOX_5,
    UBLOX_6,
    UBLOX_7,
    UBLOX_M8
};

#define UBX_PREAMBLE1 0xb5
#define UBX_PREAMBLE2 0x62

typedef struct __attribute__((packed)) {
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
} ubx_header_t;

STATIC_ASSERT(sizeof(ubx_header_t) == 6, ubx_header_t_padded);

typedef struct __attribute__((packed)) {
    const ubx_header_t header;

    uint32_t i_tow;
    uint16_t year;
    uint8_t month, day, hour, minute, second;
    uint8_t valid;
    uint32_t t_acc;
    int32_t nano;
    uint8_t fix_type;
    uint8_t flags;
    uint8_t reserved1;
    uint8_t num_sv;
    int32_t lon, lat;
    int32_t height, h_msl;
    uint32_t h_acc, v_acc;
    int32_t velN, velE, velD, gspeed;
    int32_t head_mot;
    uint32_t s_acc;
    uint32_t head_acc;
    uint16_t p_dop;
    uint16_t reserved2;
    uint32_t reserved3;
    int32_t head_veh;
    uint32_t reserved4;

    uint8_t ck_a, ck_b;
} ubx_nav_pvt_t;

#define UBX_CFG_MSG_HEADER {.preamble1 = UBX_PREAMBLE1, .preamble2 = UBX_PREAMBLE2, .msg_id = UBX_CFG_MSG, .msg_class = UBX_CLASS_CFG, .length = 3}

typedef struct __attribute__((packed)) {
    const ubx_header_t header;

    uint8_t cfg_msg_class;
    uint8_t cfg_msg_id;
    uint8_t rate;

    uint8_t ck_a, ck_b;
} ubx_cfg_msg_t;

#define UBX_CFG_RATE_HEADER {.preamble1 = UBX_PREAMBLE1, .preamble2 = UBX_PREAMBLE2, .msg_id = UBX_CFG_RATE, .msg_class = UBX_CLASS_CFG, .length = 6}

typedef struct __attribute__((packed)) {
    const ubx_header_t header;

    uint16_t meas_rate;
    uint16_t nav_rate;
    uint16_t time_ref;

    uint8_t ck_a, ck_b;
} ubx_cfg_rate_t;


#define UBX_CFG_NAV5_HEADER {.preamble1 = UBX_PREAMBLE1, .preamble2 = UBX_PREAMBLE2, .msg_id = UBX_CFG_NAV5, .msg_class = UBX_CLASS_CFG, .length = 36}

typedef struct __attribute__((packed)) {
    const ubx_header_t header;

    uint16_t mask;
    uint8_t dyn_model;
    uint8_t fix_mode;
    int32_t fixed_alt;
    uint32_t fixed_alt_var;
    int8_t min_elev;
    uint8_t dr_limit;
    uint16_t p_dop, t_dop, p_acc, t_acc;
    uint8_t static_hold_thres;
    uint8_t dgps_timeout;
    uint8_t cno_thresh_num_svs, cno_thresh;
    uint16_t reserved;
    uint16_t static_hold_max_dist;
    uint8_t utc_standard;
    uint8_t reserved3;
    uint32_t reserved4;

    uint8_t ck_a, ck_b;
} ubx_cfg_nav5_t;

typedef struct __attribute__((packed)) {
    const ubx_header_t header;
    uint8_t ack_msg_class, ack_msg_id;
    uint8_t ck_a, ck_b;
} ubx_ack_t;

#endif /* UBLOX_UBX_H */