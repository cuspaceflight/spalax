#include "CanSerialDriver.h"
#include "serial_interface.h"
#include "messaging.h"
#include <avionics_config.h>
#include "messaging_config.h"
#include <serial/serial.h>
#include <component_state.h>
#include "can_interface.h"

#define READ_BUFFER_SIZE 255
#define WRITE_BUFFER_SIZE 255

static unsigned int read_buffer_index = 0;
static unsigned int write_buffer_index = 0;

static unsigned int read_buffer_limit = 0;
static uint8_t read_buffer[READ_BUFFER_SIZE];
static uint8_t write_buffer[WRITE_BUFFER_SIZE];

static bool is_initialised = false;
static CanSerialDriver* s_instance = nullptr;
static serial::Serial* s_port = nullptr;

static uint8_t stream_get() {
	while (read_buffer_index >= read_buffer_limit) {
		if (s_port == nullptr) {
			// Trigger termination of the read
			return 0x7E;
		}
		read_buffer_limit = s_port->read(read_buffer, READ_BUFFER_SIZE);
		read_buffer_index = 0;
		if (read_buffer_index >= read_buffer_limit)
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
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

struct can_header_t {
	uint16_t id;
	uint8_t rtr;
	uint8_t dlc;
};


SERIAL_INTERFACE(serial_interface, stream_get, stream_put, stream_flush, 1024);

void can_send(uint16_t msg_id, bool can_rtr, uint8_t* data, uint8_t datalen) {
	can_header_t header;
	header.id = msg_id;
	header.rtr = can_rtr ? 1 : 0;
	header.dlc = datalen;

	if (!serial_interface.stream_put(0x7E)) {
		return;
	}
	
	if (!serial_interface_write_bytes_to_buffer(&serial_interface, (uint8_t*)&header, sizeof(header))) {
		return;
	}

	serial_interface_write_bytes_to_buffer(&serial_interface, data, datalen);
}

static bool receive_packet(const telemetry_t* packet, message_metadata_t metadata) {
	if (packet->header.origin == local_config.origin) {
		can_send_telemetry(packet, metadata);
	}

	return true;
}


MESSAGING_CONSUMER(messaging_consumer, 0, 0, message_flags_send_over_can, message_flags_send_over_can, receive_packet, 100);

static void reader_thread(CanSerialDriver* driver) {
	while (s_port != nullptr) {
		messaging_consumer_receive(&messaging_consumer, true, false);
	}
}

bool read_can_frame() {
	can_header_t header;
	if (!serial_interface_read_bytes_to_buffer(&serial_interface, (uint8_t*)&header, sizeof(header))) {
		return false;
	}
	uint8_t data[8];
	if (!serial_interface_read_bytes_to_buffer(&serial_interface, data, header.dlc)) {
		return false;
	}

	can_recv(header.id, header.rtr == 1 ? true : false, data, header.dlc);
	return true;
}

static bool handle_next_can_packet() {
	while (serial_interface.stream_get() != 0x7E) {}
	if (!read_can_frame())
		return false;
	return true;
}

static void writer_thread(CanSerialDriver* driver) {
	while (s_port != nullptr) {
		handle_next_can_packet();
	}
}

CanSerialDriver::CanSerialDriver(const char* port_name, int baud_rate) {
	FTAssert(!is_initialised, "Only one serial driver can be active at once");


	serial::Timeout timeout(0, 10, 0, 0, 0);

	try {
		serial_port_ = std::make_unique<serial::Serial>(port_name, baud_rate, timeout);
		read_buffer_index = 0;
		write_buffer_index = 0;
	} catch (serial::IOException ex) {
		serial_port_ = nullptr;
	}

	if (serial_port_ != nullptr && serial_port_->isOpen()) {
		s_instance = this;

		is_initialised = true;
		s_port = serial_port_.get();

		messaging_consumer_init(&messaging_consumer);
		serial_interface_init(&serial_interface);

		writer_thread_ = std::thread(writer_thread, this);
		reader_thread_ = std::thread(reader_thread, this);
	}
}

CanSerialDriver::~CanSerialDriver() {
	if (!is_initialised)
		return; // If initialisation failed we don't have anything to clean up

	is_initialised = false;
	s_port = nullptr;
	s_instance = nullptr;
	serial_port_->close();
	serial_port_.reset();
	writer_thread_.join();
	reader_thread_.join();
}
