#pragma once

#include "Comms/Packet.hpp"

class PacketBuffer {
    private:
        static const size_t _SIZE =                          // Number of different packet types (ignoring TERMINATE)
            static_cast<size_t>(Action::COUNT) - 1;
        Packet* _packets[_SIZE];                             // Array of packet references

    public:
        PacketBuffer() = default;

        /***
         * @brief Inserts a packet into the buffer
         * 
         * @param packet    The packet being inserted
         */
        void insert(const Packet& packet);

        /***
         * @brief Clears the packet buffer
         */
        void clear();

        /***
         * @brief Fetches the packet with the given action
         * 
         * @return Immutable pointer to the desired packet
         */
        Packet* const operator[](Action action);
};

void PacketBuffer::insert(const Packet& packet) {
    Action action = packet.action();
    size_t index = (size_t)(action) - 1;

    // Store packet in buffer
    if (action != Action::TERMINATE) {
        if (_packets[index])
            delete _packets[index];
        _packets[index] = new Packet(packet);
    }
}

void PacketBuffer::clear() {
    for (size_t i = 0; i < _SIZE; i++) {
        if (_packets[i]) {
            delete _packets[i];
            _packets[i] = nullptr;
        }
    }
}

Packet* const PacketBuffer::operator[](Action action) {
    return _packets[(size_t)(action) - 1];
}