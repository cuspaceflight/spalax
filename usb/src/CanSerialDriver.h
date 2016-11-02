#pragma once
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

void can_send(uint16_t msg_id, bool can_rtr, uint8_t* data, uint8_t datalen);