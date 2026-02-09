#ifndef PACKET_BUFFER_HPP
#define PACKET_BUFFER_HPP

#include "Comms/Packet.hpp"

class PacketBuffer {
    private:
        static const size_t _SIZE =                          // Number of different packet types (ignoring TERMINATE)
            static_cast<size_t>(Action::COUNT) - 1;
        Packet* _packets[_SIZE];                             // Array of packet references

    public:
        using iterator = Packet**;
        using const_iterator = const iterator;

        /***
         * @brief Creates a new PacketBuffer
         */
        PacketBuffer() {
            for (size_t i = 0; i < _SIZE; ++i) {
                _packets[i] = nullptr;
            }
        }

        /***
         * @brief Inserts a packet into the buffer
         * 
         * @param packet    The packet being inserted
         */
        void insert(const Packet& packet) {
            // Fetch packet action
            Action action = packet.action();

            // Ignore any control packet types
            if (action < Action::TERMINATE) {
                size_t index = (size_t)(action);
                if (_packets[index])
                    delete _packets[index];
                _packets[index] = new Packet(packet);
            }
        }

        /***
         * @brief Clears the packet buffer
         */
        void clear() {
            for (size_t i = 0; i < _SIZE; i++) {
                if (_packets[i]) {
                    delete _packets[i];
                    _packets[i] = nullptr;
                }
            }
        }

        /***
         * @brief Fetches the packet with the given action
         * 
         * @return Pointer to the desired packet
         */
        Packet* operator[](Action action) {
            return _packets[(size_t)(action)];
        }

        /***
         * @brief Fetches the packet with the given action
         * 
         * @return Immutable pointer to the desired packet
         */
        const Packet* operator[](Action action) const {
            return _packets[(size_t)(action)];
        }

        /***
         * @brief Fetches a constant iterator to the beginning of the buffer
         * 
         * @return Constant iterator pointing to the beginning of the buffer
         */
        const_iterator cbegin() const {
            return (const_iterator) _packets;
        }

        /***
         * @brief Fetches an iterator to the end of the buffer
         * 
         * @return Constant iterator pointing to the end of the buffer
         */
        const_iterator cend() const {
            return (const_iterator) (_packets + _SIZE);
        }

        /***
         * @brief Fetches an iterator to the beginning of the buffer
         * 
         * @return Iterator pointing to the beginning of the buffer
         */
        const_iterator begin() const {
            return begin();
        }

        /***
         * @brief Fetches an iterator to the end of the buffer
         * 
         * @return Iterator pointing to the end of the buffer
         */
        const_iterator end() const {
            return end();
        }

        /***
         * @brief Fetches an iterator to the beginning of the buffer
         * 
         * @return Iterator pointing to the beginning of the buffer
         */
        iterator begin() {
            return _packets;
        }

        /***
         * @brief Fetches an iterator to the end of the buffer
         * 
         * @return Iterator pointing to the end of the buffer
         */
        iterator end() {
            return _packets + _SIZE;
        }
};

#endif