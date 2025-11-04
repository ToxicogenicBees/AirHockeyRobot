#include "Comms/SerialLink.h"
#include "Constants.h"

#include <Arduino.h>

void SerialLink::init() {
    Serial.begin(Constants::Comms::BAUD_RATE);  // Initialize the serial communication with the desired baud rate
    while (!Serial);                            // Wait for Serial to be trully initialized
}

void SerialLink::send(Packet& packet) {
    // Ensure the packet is finalized
    packet.finalize();

    // Send the raw bytes across the link
    Serial.write(packet._data.data(), packet.length());
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