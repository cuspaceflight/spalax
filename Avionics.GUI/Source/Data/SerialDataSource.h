#pragma once
#include <Data/TelemetrySerialPort.h>
#include "DataSource.h"

class SerialDataSource : public DataSource
{
public:
	SerialDataSource(const char* port_name, int baud);
	virtual ~SerialDataSource();

	void update(uint64_t dt, state_estimate_t* estimate) override;

	void nextPacket(const telemetry_t& packet);

private:
	std::unique_ptr<TelemetrySerialPort> serial_port_;
	bool packet_waiting_;
	telemetry_t waiting_packet_;
};

