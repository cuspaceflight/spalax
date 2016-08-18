#pragma once
#include <Rendering/FTScene.h>
#include "SerialPort.h"
#include <thread>

class SerialDriver {
public:
    explicit SerialDriver(const char* port_name, int baud_rate);
    ~SerialDriver();
private:
    std::unique_ptr<SerialPort> serial_port_;

    std::thread reader_thread_;
    std::thread writer_thread_;
};
