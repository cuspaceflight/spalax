#include "TelemetrySerialPort.h"

#define BUFFER_COUNT 1

TelemetrySerialPort::TelemetrySerialPort(const char* port_name, int baud_rate) : SerialPort(port_name, baud_rate), message_buffer_index_(-1) {
}


TelemetrySerialPort::~TelemetrySerialPort() {
	
}

// Keep sending SYNC to the board until we start getting data back
void TelemetrySerialPort::sync() {
	int n = 0;
	const char characters[4] = { 'S', 'Y','N','C'};
	while (!poll()) {
		write((const uint8_t*)characters, 4);
		Sleep(1000);
	}
}

bool TelemetrySerialPort::poll() {
	bool ret = false;
	uint8_t buffer[1024];
	uint8_t* message_buffer_as_bytes = (uint8_t*)&message_read_buffer_;
	int n = read(buffer, 1024);

	for (int i = 0; i < n; i++) {
		if (message_buffer_index_ == -2) {
			if (buffer[i] == 'S')
				message_buffer_index_ = -1;
			continue;
		}
		else if (message_buffer_index_ == -1) {
			if (buffer[i] == 'Y')
				message_buffer_index_ = 0;
			else
				message_buffer_index_ = -2;
			continue;
		}
		else {
			message_buffer_as_bytes[message_buffer_index_++] = buffer[i];
			if (message_buffer_index_ == sizeof(telemetry_t)) {
				message_buffer_index_ = -2;
				// TODO check checksum of packet
				if (message_read_buffer_.check_sum_ == 0)
					message_cache_.push(message_read_buffer_);
				else
					FTLOG("Packet failed Checksum");
				ret = true;
			}
		}
	}
	

	return ret;
}

bool TelemetrySerialPort::getNext() {
	if (message_cache_.empty() && !poll())
		return false;
	packet_received_delegate_(message_cache_.front());
	message_cache_.pop();
	return true;
}
