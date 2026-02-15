#ifndef STMLINK_HPP
#define STMLINK_HPP

#include "Comms/SerialLink.hpp"
#include "Constants.h"
#include <Arduino.h>

class STMLink : public SerialLink {
    private:
        static void _sendPacket(const Packet& packet) {
            Serial.write(packet.data().data(), packet.length());
        }

        static Packet _receivePacket() {
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
                return Packet(Action::INVALID);
            }

            // Return packet
            return packet;
        }

        static void _initLink() {
            Serial.begin(Constants::Comms::BAUD_RATE);  // Initialize the serial communication with the desired baud rate
            while (!Serial);                            // Wait for Serial to be trully initialized
        }

};


#endif
