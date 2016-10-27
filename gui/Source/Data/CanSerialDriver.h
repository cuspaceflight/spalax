#pragma once
#include <Rendering/FTScene.h>
#include <thread>

namespace serial {
class Serial;
}

class CanSerialDriver {
public:
    explicit CanSerialDriver(const char* port_name, int baud_rate);
    ~CanSerialDriver();
private:
    std::unique_ptr<serial::Serial> serial_port_;

    std::thread reader_thread_;
    std::thread writer_thread_;
};
