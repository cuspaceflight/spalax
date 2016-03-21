#include <telemetry.h>
#include <logging.h>

void telemetry_print_data(const telemetry_t* data) {
    PRINT("Telemetry Packet - Origin: %i, Source: %i, Tag: %i, Timestamp: %i and ", data->origin, data->source, data->tag, data->timestamp);
    switch (data->mode) {
    case telemetry_mode_string:
        PRINT("String Data: ");
        for (int i = 0; i < 8; ++i)
            PRINT("%c", data->string_data[i]);
        break;
    case telemetry_mode_int64:
        PRINT("Int64 Data:");
        for (int i = 0; i < 2; ++i)
            PRINT(" %ld", data->uint64_data[i]);
        break;
    case telemetry_mode_uint64:
        PRINT("UInt64 Data:");
        for (int i = 0; i < 2; ++i)
            PRINT(" %ul", data->uint64_data[i]);
        break;
    case telemetry_mode_int32:
        PRINT("Int32 Data:");
        for (int i = 0; i < 4; ++i)
            PRINT(" %i", data->int32_data[i]);
        break;
    case telemetry_mode_uint32:
        PRINT("UInt32 Data:");
        for (int i = 0; i < 4; ++i)
            PRINT(" %i", data->uint32_data[i]);
        break;
    case telemetry_mode_int16:
        PRINT("Int16 Data:");
        for (int i = 0; i < 8; ++i)
            PRINT(" %i", data->int16_data[i]);
        break;
    case telemetry_mode_uint16:
        PRINT("UInt16 Data:");
        for (int i = 0; i < 8; ++i)
            PRINT(" %i", data->uint16_data[i]);
        break;
    case telemetry_mode_int8:
        PRINT("Int8 Data:");
        for (int i = 0; i < 16; ++i)
            PRINT(" %i", data->int8_data[i]);
        break;
    case telemetry_mode_uint8:
        PRINT("UInt8 Data:");
        for (int i = 0; i < 16; ++i)
            PRINT(" %i", data->uint8_data[i]);
        break;
    case telemetry_mode_float:
        PRINT("Float Data:");
        for (int i = 0; i < 4; ++i)
            PRINT(" %f", data->float_data[i]);
        break;
    case telemetry_mode_double:
        PRINT("Double Data:");
        for (int i = 0; i < 2; ++i)
            PRINT(" %f", data->double_data[i]);
        break;
    default:
        PRINT("Unrecognised Packet Mode %i", data->mode);
    }
    PRINT("\n");
}