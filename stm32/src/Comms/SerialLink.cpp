#include "Comms/SerialLink.hpp"
#include "Constants.h"

#include <Arduino.h>

PacketBuffer SerialLink::_receive_buffer;
PacketBuffer SerialLink::_send_buffer;
bool SerialLink::_ready = false;

void SerialLink::_sendOverLink(const Packet& packet) {
    Serial.write(packet._data.data(), packet.length());
}

void SerialLink::init() {
    Serial.begin(Constants::Comms::BAUD_RATE);  // Initialize the serial communication with the desired baud rate
    while (!Serial);                            // Wait for Serial to be trully initialized
}

void SerialLink::buffer(const Packet& packet) {
    _send_buffer.insert(packet);
}

void SerialLink::send() {
    // Send all buffered packets
    for (auto p : _send_buffer) {
        if (p) {
            p->finalize();
            _sendOverLink(*p);
        }
    }

    // Send a sentinal packet to signal end of communication
    Packet terminate(Action::TERMINATE);
    terminate.finalize();
    _sendOverLink(terminate);

    // Clear the packet buffer
    _send_buffer.clear();
}

Packet SerialLink::read() {
    // Wait for a byte to be sent over
    while (Serial.available() < 1);

    // Get the length of the new packet
    uint8_t length = Serial.read();

    // Create a buffer for this packet
    std::vector<uint8_t> buffer(length);
    buffer[0] = length;

    // Read the incoming packet byte-by-byte
    size_t index = 1;
    while (index < length) {
        if (Serial.available()) {
            buffer[index++] = Serial.read();
        }
    }

    // Construct the packet object
    Packet packet(buffer);

    // Handle invalid packet
    if (!packet.isValid()) {
        
    }

    // Return packet
    return packet;
}