#ifndef SERIAL_LINK_HPP
#define SERIAL_LINK_HPP

#include "Comms/PacketBuffer.hpp"
#include "Comms/Packet.hpp"
#include "Types/Timer.hpp"
#include "Constants.h"

#include <Arduino.h>
#include <functional>
#include <stdint.h>
#include <vector>

class SerialLink {
    private:
        using processor = std::function<void(Packet&)>;

        // Incoming bytes buffer
        static std::vector<uint8_t> _rx_buffer;

        // Buffered completed packets
        static PacketBuffer _receive_buffer;
        static PacketBuffer _send_buffer;

        // Packet handler
        static processor _callback;

        // Communication timer
        static Timer _timer;

        // Receive a packet from the link
        static Packet _receivePacket() {
            // Store all incoming bytes
            while (Serial.available())
                _rx_buffer.push_back(Serial.read());

            // Check if incoming bytes are long enough to form a valid packet
            if (!_rx_buffer.size() || _rx_buffer.size() < _rx_buffer[0])
                return {Action::Invalid};

            // Fetch packet
            Packet packet(_rx_buffer);

            // Validate packet
            if (!packet.isValid())
                return {Action::Invalid};
            
            // Return packet
            _rx_buffer.erase(_rx_buffer.begin(), _rx_buffer.begin() + packet.length());
            return packet;
        }

    public:
        static void init(processor callback) {
            // Store packet callback function
            _callback = callback;

            // Initialize serial link
            Serial.begin(Constants::Comms::BAUD_RATE);
            while (!Serial);

            // Reset timer
            _timer.reset();
        }

        static void buffer(const Packet& packet) {
            _send_buffer.insert(packet);
        }

        static void process() {
            // Fetch incoming packet
            bool receivedTermination = false;
            Packet packet;

            do {
                packet = _receivePacket();

                if (packet.action() != Action::Invalid) {
                    // Store in buffer
                    _receive_buffer.insert(packet);

                    // If this was the last packet
                    if (packet.action() == Action::Terminate) {
                        // Process all packets
                        for (auto p : _receive_buffer) {
                            if (p) {
                                _callback(*p);
                            }
                        }
                        _receive_buffer.clear();
                        receivedTermination = true;
                    }
                }

            } while (packet.action() != Action::Invalid && _timer.delta<std::chrono::seconds>() < Constants::Comms::TIMEOUT);

            if (receivedTermination) {
                // Send buffered packets
                for (auto p : _send_buffer) {
                    if (p) {
                        p->finalize();
                        Serial.write(p->data(), p->length());
                    }
                }
                _send_buffer.clear();

                // Send sentinal packet
                Packet terminate(Action::Terminate);
                terminate.finalize();
                Serial.write(terminate.data(), terminate.length());

                // Reset timer
                _timer.reset();
            }

            else if (_timer.delta<std::chrono::seconds>() >= Constants::Comms::TIMEOUT) {
                // Reset incoming buffer
                _rx_buffer.clear();

                // Reset timer
                _timer.reset(); 
            }
        }
};

std::vector<uint8_t> SerialLink::_rx_buffer;
PacketBuffer SerialLink::_receive_buffer;
PacketBuffer SerialLink::_send_buffer;
SerialLink::processor SerialLink::_callback;
Timer SerialLink::_timer;

#endif
