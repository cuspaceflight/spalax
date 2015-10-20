#ifndef TELEMETRY_H
#define TELEMETRY_H
#include <stdint.h>

typedef struct telemetry_t{
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

#define MODE_STRING 0
#define MODE_INT64 1
#define MODE_UINT64 2
#define MODE_INT32 3
#define MODE_UINT32 4
#define MODE_INT16 5
#define MODE_UINT16 6
#define MODE_INT8 7
#define MODE_UINT8 8
#define MODE_FLOAT 9
#define MODE_DOUBLE 10

#define PACKET_ACCEL_RAW 0x20
#define PACKET_MAG_RAW 0x23
#define PACKET_GYRO_RAW 0x24
#define PACKET_PRESSURE_RAW 0x22

void print_telemetry_data(const telemetry_t* data);
#endif /* TELEMETRY_H */