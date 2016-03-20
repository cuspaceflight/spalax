#include <telemetry.h>
#include <logging.h>

void print_telemetry_data(const telemetry_t* data) {
    telemetry_mode_t mode = data->metadata_ & 0xF;

	PRINT("Packet from origin: %i on channel: 0x%X with timestamp: %i and ", mode, data->channel_, data->timestamp_);
	switch (mode) {
	case telemetry_mode_string:
		PRINT("String Data: ");
		for (int i = 0; i < 8; ++i)
			PRINT("%c", data->string_data_[i]);
		break;
	case telemetry_mode_int64:
		PRINT("Int64 Data: %ld", data->int64_data_);
		break;
	case telemetry_mode_uint64:
		PRINT("UInt64 Data: %ul", data->uint64_data_);
		break;
	case telemetry_mode_int32:
		PRINT("Int32 Data:");
		for (int i = 0; i < 2; ++i)
			PRINT(" %i", data->int32_data_[i]);
		break;
	case telemetry_mode_uint32:
		PRINT("UInt32 Data:");
		for (int i = 0; i < 2; ++i)
			PRINT(" %i", data->uint32_data_[i]);
		break;
	case telemetry_mode_int16:
		PRINT("Int16 Data:");
		for (int i = 0; i < 4; ++i)
			PRINT(" %i", data->int16_data_[i]);
		break;
	case telemetry_mode_uint16:
		PRINT("UInt16 Data:");
		for (int i = 0; i < 4; ++i)
			PRINT(" %i", data->uint16_data_[i]);
		break;
	case telemetry_mode_int8:
		PRINT("Int8 Data:");
		for (int i = 0; i < 8; ++i)
			PRINT(" %i", data->int8_data_[i]);
		break;
	case telemetry_mode_uint8:
		PRINT("UInt8 Data:");
		for (int i = 0; i < 8; ++i)
			PRINT(" %i", data->uint8_data_[i]);
		break;
	case telemetry_mode_float:
		PRINT("Float Data:");
		for (int i = 0; i < 2; ++i)
			PRINT(" %i", data->float_data_[i]);
		break;
	case telemetry_mode_double:
		PRINT("Double Data: %f", data->double_data_);
		break;
	default:
		PRINT("Unrecognised Packet Mode %i", mode);
	}
	PRINT("\n");
}
