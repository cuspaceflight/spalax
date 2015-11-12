#pragma once
#include "SerialPort.h"
#include <ThirdParty/Signals/Delegate.h>
#include <queue>

extern "C" {
#include <telemetry.h>
}

class TelemetrySerialPort : public SerialPort {
public:
	TelemetrySerialPort(const char* port_name, int baud_rate);
	~TelemetrySerialPort();

	Gallant::Delegate1<const telemetry_t&>* getPacketReceivedDelegate() {
		return &packet_received_delegate_;
	}
	
	// Call the delegate with the next packet and return true if there is one otherwise return false
	bool getNext();

	void sync();

protected:
	virtual int read(uint8_t* buff, size_t size) override {
		return SerialPort::read(buff, size);
	}

	// Retrieve new packets from the serial port
	bool poll();

	Gallant::Delegate1<const telemetry_t&> packet_received_delegate_;
	telemetry_t message_read_buffer_;
	int8_t message_buffer_index_;
	std::queue<telemetry_t> message_cache_;
};

