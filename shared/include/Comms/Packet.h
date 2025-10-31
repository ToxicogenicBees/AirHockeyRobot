#pragma once

#include <stdexcept>
#include <stdint.h>
#include <cstring>
#include <vector>

/*

Packet structure:

    A L MMM...MMM C

Byte 0:                 Action
Byte 1:                 Length
Bytes 2 - (N - 1):      Message
Byte N:                 CRC

*/

namespace Action {
    constexpr uint8_t MALLET_POSITION = 0x00;
}

class Packet {
    private:
        std::vector<uint8_t> _data;
        std::vector<uint8_t>::iterator _read_iter = _data.begin();

    public:
        /***
         * @brief Creates a packet with the desired action type
         * 
         * @param action    The desired action
         */
        Packet(uint8_t action = Action::MALLET_POSITION);

        /***
         * @brief Resets the read iterator to the first byte of message data
         */
        void resetRead();

         /***
         * @brief Runs final calculations to prep packet for sending
         */
        void finalize();

        /**
         * @brief Determines if the CRC in the packet matches with the data inside the packet
         * 
         * @return If the packet is valid
         */
        bool isValid();

        /**
         * @brief Returns the packet's action
         * 
         * @return The packet's action
         */
        uint8_t action() { return _data[0]; }

        /**
         * @brief Returns the packet's length
         * 
         * @return The packet's length
         */
        uint8_t length() { return _data[1]; }

        /**
         * @brief Returns the packet's stored CRC
         * 
         * @return The packet's stored CRC
         */
        uint8_t crc() { return _data[_data.size() - 1]; }

        /**
         * @brief Writes the given data into the packet's payload
         * 
         * @param val   The value to be written
         */
        template<class T>
        void write(const T& val) {
            auto ptr = reinterpret_cast<const uint8_t*>(&val);
            _data.insert(_data.end(), ptr, ptr + sizeof(T));
        }
        
        /**
         * @brief Writes the given data into the packet's payload
         * 
         * @param val   The value to be written
         * 
         * @return  A reference to this packet
         */
        template<class T>
        Packet& operator<<(const T& val) {
            write<T>(val);
            return *this;
        }
        
        /**
         * @brief Reads a value of the specified type from the bytes stored in the payload, from
         *        the last read position from the packet
         *        Throws std::out_of_range if there aren't enough bytes in the packet to read
         * 
         * @return  The read value
         */
        template<class T>
        T read() {
            return read<T>(_read_iter);
        }
        
        /**
         * @brief Reads a value of the specified type from the bytes stored in the payload
         *        Throws std::out_of_range if there aren't enough bytes in the packet to read
         * 
         * @param iter  The desired location to read from
         * 
         * @return  The read value
         */
        template<class T, class Iter>
        T read(Iter iter) {
            if (std::distance(iter, _data.end()) < sizeof(T))
                throw std::out_of_range("Not enough bytes in packet to read value");

            T value;
            std::memcpy(&value, &(*iter), sizeof(T));

            _read_iter = iter + sizeof(T);
            return value;
        }
};