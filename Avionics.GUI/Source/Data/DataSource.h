#pragma once
#include <stdint.h>
#include <Frontier.h>
extern "C" {
#include <state_estimate.h>
#include <telemetry.h>
}

class DataSource {
public:
	virtual void update(uint64_t dt, state_estimate_t* estimate) {
		simulation_time_ += dt;
	}

    virtual ~DataSource();

protected:
	DataSource();
	

	bool isPacketInFuture(const telemetry_t& packet);

	void handlePacket(const telemetry_t& packet);

	void setStartOffset(uint64_t offset) {
		simulation_time_ = offset;
		last_packet_time_ = (uint32_t)offset;
	}

private:
	uint64_t simulation_time_;
	uint64_t packet_timestep_correction_;
	uint32_t last_packet_time_;
};