#pragma once
#include <thread>

namespace serial {
class Serial;
}

class SerialDriver {
public:
    explicit SerialDriver(const char* port_name, int baud_rate);
    ~SerialDriver();

    bool getInitialized();
private:
    std::unique_ptr<serial::Serial> serial_port_;

    std::thread reader_thread_;
    std::thread writer_thread_;
};
