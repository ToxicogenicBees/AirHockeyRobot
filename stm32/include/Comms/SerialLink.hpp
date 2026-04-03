#pragma once

#include "Comms/PacketBuffer.hpp"
#include "Comms/Packet.hpp"
#include "Types/Timer.hpp"
#include "Constants.hpp"

#include <Arduino.h>
#include <functional>
#include <stdint.h>
#include <unordered_map>
#include <vector>

class SerialLink {
    private:
        using processor = std::function<void(Packet&)>;

        // Incoming bytes buffer
        inline static std::vector<uint8_t> _rx_buffer;

        // Buffered completed packets
        inline static PacketBuffer _receive_buffer;
        inline static PacketBuffer _send_buffer;

        // Packet handlers
        inline static std::unordered_map<Action, processor> _handlers;

        // Communication timer
        inline static Timer _timer;

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
        static void init() {
            // Initialize serial link
            Serial.begin(Constants::Comms::BAUD_RATE);
            while (!Serial);

            // Reset timer
            _timer.reset();
        }

        static void registerHandler(Action action, processor handler) {
            _handlers[action] = handler;
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
                        for (auto& p : _receive_buffer) {
                            if (p) {
                                auto callback = _handlers.find(p->action());
                                if (callback != _handlers.end()) {
                                    p->resetRead();
                                    callback->second(*p);
                                }
                            }
                        }
                        _receive_buffer.clear();
                        receivedTermination = true;
                    }
                }

            } while (packet.action() != Action::Invalid && _timer.delta<std::chrono::seconds>() < Constants::Comms::TIMEOUT);

            if (receivedTermination) {
                // Send buffered packets
                for (auto& p : _send_buffer) {
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
