#ifndef TELEMETRY_H
#define TELEMETRY_H
#include <stdint.h>

typedef enum {
    telemetry_mode_string = 0,
    telemetry_mode_int64 = 1,
    telemetry_mode_uint64 = 2,
    telemetry_mode_int32 = 3,
    telemetry_mode_uint32 = 4,
    telemetry_mode_int16 = 5,
    telemetry_mode_uint16 = 6,
    telemetry_mode_int8 = 7,
    telemetry_mode_uint8 = 8,
    telemetry_mode_float = 9,
    telemetry_mode_double = 10,
} telemetry_mode_t;

typedef enum {
    telemetry_source_system = 0,
    telemetry_source_calibration = 1,
    telemetry_source_imu = 2,
    telemetry_source_state_estimation = 5,
    telemetry_source_wildcard = 15
} telemetry_source_t;

typedef struct telemetry_t {
	uint32_t timestamp_;
	uint8_t metadata_;
    uint8_t channel_;
	uint16_t check_sum_;

	union {
		char string_data_[8];
		int64_t int64_data_;
		uint64_t uint64_data_;
		int32_t int32_data_[2];
		uint32_t uint32_data_[2];
		int16_t int16_data_[4];
		uint16_t uint16_data_[4];
		int8_t int8_data_[8];
		uint8_t uint8_data_[8];
		float float_data_[2];
		double double_data_;
	};
} telemetry_t;

#define PACKET_ACCEL_RAW 0x20
#define PACKET_MAG_RAW 0x23
#define PACKET_GYRO_RAW 0x24
#define PACKET_PRESSURE_RAW 0x22

telemetry_mode_t telemetry_get_mode(const telemetry_t* data);

telemetry_source_t telemetry_get_source(const telemetry_t* data);

uint8_t telemetry_get_packet_id(const telemetry_t* data);

void telemetry_print_data(const telemetry_t* data);

#endif /* TELEMETRY_H */
