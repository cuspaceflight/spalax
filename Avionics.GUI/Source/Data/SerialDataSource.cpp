#include "SerialDataSource.h"

SerialDataSource::SerialDataSource(const char* port_name, int baud) : serial_port_(new TelemetrySerialPort(port_name,baud)) {
    serial_port_->getPacketReceivedDelegate()->Bind(this, &SerialDataSource::nextPacket);
    serial_port_->sync();
    serial_port_->getNext();
    FTAssert(packet_waiting_, "No Packet waiting");
    setStartOffset(waiting_packet_.timestamp);
}


SerialDataSource::~SerialDataSource() {
}

void SerialDataSource::update(uint64_t dt, state_estimate_t* estimate) {
    DataSource::update(dt,estimate);
    
    if (packet_waiting_ && isPacketInFuture(waiting_packet_)) {
        return;
    }
    handlePacket(waiting_packet_);
    packet_waiting_ = false;
    do {
        if (!serial_port_->getNext())
            break;
    } while (!packet_waiting_);
}

void SerialDataSource::nextPacket(const telemetry_t& packet) {
    if (isPacketInFuture(packet)) {
        waiting_packet_ = packet;
        packet_waiting_ = true;
    }
    else
        handlePacket(packet);
}