#include "DataSource.h"
extern "C" {
#include <state_estimate.h>
#include <mission.h>
#include "time_utils.h"
}

DataSource::DataSource() : simulation_time_(0), packet_timestep_correction_(0), last_packet_time_(0) {
	FTLOG("Data Source created");
}

DataSource::~DataSource() {

}

bool DataSource::isPacketInFuture(const telemetry_t& data) {
	if (data.timestamp < last_packet_time_) {
		FTLog("Data Source uint32_t clock rollover");
		packet_timestep_correction_ += 0xFFFFFFFF;
	}
	last_packet_time_ = data.timestamp;
	uint64_t packet_true_time = data.timestamp + packet_timestep_correction_;

	if (packet_true_time > simulation_time_) {
		return true;
	}
	return false;
}

void DataSource::handlePacket(const telemetry_t& data) {
    //TODO - reimplement using messaging system
    platform_set_counter_value(data.timestamp);
//	switch (data.channel_) {
//	case PACKET_PRESSURE_RAW: {
//		state_estimate_new_pressure_raw(data.int32_data_[0]);
//		break;
//	}
//	case PACKET_ACCEL_RAW: {
//		state_estimate_new_accel_raw(data.int16_data_);
//		break;
//	}
//	case PACKET_GYRO_RAW: {
//		state_estimate_new_gyro_raw(data.int16_data_);
//		break;
//	}
//	case PACKET_MAG_RAW: {
//		state_estimate_new_magnetometer_raw(data.int16_data_);
//		break;
//	}
//	case 0x40: {
//		print_state_transition(data.int32_data_[0], data.int32_data_[1]);
//		break;
//	}
//	default:
//		break;
//	}
	//FTLOG("Packet %lu %lu", data.timestamp_, simulation_time_);
}