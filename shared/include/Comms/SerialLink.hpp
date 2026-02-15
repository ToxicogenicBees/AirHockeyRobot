#ifndef SERIALLINK_HPP
#define SERIALLINK_HPP

#include "Comms/PacketBuffer.hpp"
#include "Comms/Packet.hpp"
#include <functional>

class SerialLink {
    public:
        using processor = std::function<void(Packet&)>;

    private:
        static PacketBuffer _receive_buffer;
        static PacketBuffer _send_buffer;
        static processor _callback;
        static bool _ready;

        // Send a packet over the link
        static void _sendPacket(const Packet& packet) {}

        // Receive a single packet from the link
        static Packet _receivePacket() { return {Action::INVALID}; }

        // Initialize the link
        static void _initLink() {}

    public:
        /**
         * @brief   Initializes the communication link on the device's end
         * 
         * @param   callback    The packet processing callback function
         */
        static void init(processor callback) {
            _callback = callback;
            _initLink();
        }

        /**
         * @brief   Buffers a packet into the packet buffer
         * 
         * @param   packet  The packet being buffered
         */
        static void buffer(const Packet& packet) {
            _send_buffer.insert(packet);
        }

        /**
         * @brief   Process the receiving packet buffer if the link is ready
         */
        static void process() {
            // Ignore if preping to send
            if (_ready) return;

            // Fetch first packet from link
            auto received = _receivePacket();
            _receive_buffer.insert(received);
            
            // Continue fetching until termination
            while (received.action() != Action::TERMINATE) {
                received = _receivePacket();
                _receive_buffer.insert(received);
            }
            
            // Process packets
            for (auto p : _receive_buffer) {
                if (p) {
                    _callback(*p);
                }
            }
            _receive_buffer.clear();

            // Flag as ready to send
            _ready = true;
        }
};

PacketBuffer SerialLink::_receive_buffer;
PacketBuffer SerialLink::_send_buffer;
SerialLink::processor SerialLink::_callback;
bool SerialLink::_ready;

#endif