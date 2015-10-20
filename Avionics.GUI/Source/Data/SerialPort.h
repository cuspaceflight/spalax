#pragma once
#include <Frontier.h>
#include <Windows.h>

class SerialPort {
public:
	SerialPort(const char* port_name, int baud_rate);
	virtual ~SerialPort();

	virtual int read(uint8_t* buff, size_t size);

	virtual int write(const uint8_t* buff, size_t size);

	bool portIsOpen() {
		return port_handle_ != INVALID_HANDLE_VALUE;
	}

protected:
	HANDLE port_handle_;	
};

