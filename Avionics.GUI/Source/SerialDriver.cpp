#include "SerialDriver.h"
#include "serial_interface.h"
#include <atomic>
#include "messaging.h"

#define READ_BUFFER_SIZE 255
#define WRITE_BUFFER_SIZE 255

static unsigned int read_buffer_index = 0;
static unsigned int write_buffer_index = 0;

static unsigned int read_buffer_limit = 0;
static uint8_t read_buffer[READ_BUFFER_SIZE];
static uint8_t write_buffer[WRITE_BUFFER_SIZE];

static bool is_initialised = false;
static SerialDriver* s_instance = nullptr;
static SerialPort* s_port = nullptr;

static uint8_t stream_get() {
    if (read_buffer_index >= read_buffer_limit) {
        read_buffer_limit = s_port->read(read_buffer, READ_BUFFER_SIZE);
        read_buffer_index = 0;
    }

    FTAssert(read_buffer_index < read_buffer_limit, "Failed to fill read buffer!");
    return read_buffer[read_buffer_index++];
}

static bool stream_put(uint8_t byte) {
    if (write_buffer_index >= WRITE_BUFFER_SIZE) {
        s_port->write(write_buffer, WRITE_BUFFER_SIZE);
        write_buffer_index = 0;
    }
    write_buffer[write_buffer_index++] = byte;
    return true;
}

static bool stream_flush() {
    s_port->write(write_buffer, write_buffer_index);
    return true;
}


SERIAL_INTERFACE(serial_interface, stream_get, stream_put, stream_flush, 1024);

static void receive_packet(telemetry_t* packet, message_metadata_t flags) {
    if (packet->header.origin == local_config.origin)
        serial_interface_send_packet(&serial_interface, packet);
}


MESSAGING_CONSUMER(messaging_consumer, 0, 0, 0, message_flags_dont_send_over_usb, receive_packet, 100);

static void reader_thread(SerialDriver* driver) {
    while (true) {
        messaging_consumer_receive(&messaging_consumer, true, false);
    }
}

static void writer_thread(SerialDriver* driver) {
    while (true) {
        telemetry_t* packet = serial_interface_next_packet(&serial_interface);
        if (packet != nullptr)
            messaging_send(packet, 0);
    }
}

SerialDriver::SerialDriver(const char* port_name, int baud_rate) {
    FTAssert(!is_initialised, "Only one serial driver can be active at once");
   

    COMMTIMEOUTS port_timeouts;

    // Read operations will return if it has been more than 5ms since the last byte
    port_timeouts.ReadIntervalTimeout = 10;
    port_timeouts.ReadTotalTimeoutMultiplier = 0;
    port_timeouts.ReadTotalTimeoutConstant = 0;

    // We don't set timeouts for write operations
    port_timeouts.WriteTotalTimeoutMultiplier = 0;
    port_timeouts.WriteTotalTimeoutConstant = 0;

    serial_port_ = std::make_unique<SerialPort>(port_name, baud_rate, &port_timeouts);

    read_buffer_index = 0;
    write_buffer_index = 0;

    if (serial_port_->portIsOpen()) {
        s_instance = this;

        is_initialised = true;
        s_port = serial_port_.get();

        messaging_consumer_init(&messaging_consumer);

        writer_thread_ = std::thread(writer_thread, this);
        reader_thread_ = std::thread(reader_thread, this);
    }   
}

SerialDriver::~SerialDriver() {
    is_initialised = false;
    s_port = nullptr;
    s_instance = nullptr;
}
