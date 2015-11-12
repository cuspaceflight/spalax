#include <telemetry.h>
#include <logging.h>

void print_telemetry_data(const telemetry_t* data) {
	PRINT("Packet from origin: %i on channel: 0x%X with timestamp: %i and ", data->metadata_ >> 4, data->channel_, data->timestamp_);
	int mode = data->metadata_ & 0xF;
	switch (data->metadata_ & 0xF) {
	case MODE_STRING:
		PRINT("String Data: ");
		for (int i = 0; i < 8; ++i)
			PRINT("%c", data->string_data_[i]);
		break;
	case MODE_INT64:
		PRINT("Int64 Data: %ld", data->int64_data_);
		break;
	case MODE_UINT64:
		PRINT("UInt64 Data: %ul", data->uint64_data_);
		break;
	case MODE_INT32:
		PRINT("Int32 Data:");
		for (int i = 0; i < 2; ++i)
			PRINT(" %i", data->int32_data_[i]);
		break;
	case MODE_UINT32:
		PRINT("UInt32 Data:");
		for (int i = 0; i < 2; ++i)
			PRINT(" %i", data->uint32_data_[i]);
		break;
	case MODE_INT16:
		PRINT("Int16 Data:");
		for (int i = 0; i < 4; ++i)
			PRINT(" %i", data->int16_data_[i]);
		break;
	case MODE_UINT16:
		PRINT("UInt16 Data:");
		for (int i = 0; i < 4; ++i)
			PRINT(" %i", data->uint16_data_[i]);
		break;
	case MODE_INT8:
		PRINT("Int8 Data:");
		for (int i = 0; i < 8; ++i)
			PRINT(" %i", data->int8_data_[i]);
		break;
	case MODE_UINT8:
		PRINT("UInt8 Data:");
		for (int i = 0; i < 8; ++i)
			PRINT(" %i", data->uint8_data_[i]);
		break;
	case MODE_FLOAT:
		PRINT("Float Data:");
		for (int i = 0; i < 2; ++i)
			PRINT(" %i", data->float_data_[i]);
		break;
	case MODE_DOUBLE:
		PRINT("Double Data: %f", data->double_data_);
		break;
	default:
		PRINT("Unrecognised Packet Mode %i", data->metadata_ >> 4);
	}
	PRINT("\n");
}