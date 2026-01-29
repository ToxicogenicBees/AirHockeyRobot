#pragma once

#include <stdexcept>
#include <stdint.h>
#include <cstring>
#include <vector>

/*
    Packet structure:

    L A MMM...MMM C

    Byte 0:                 Length
    Byte 1:                 Action
    Bytes 2 - (N - 1):      Message
    Byte N:                 CRC
*/

enum class Action {
    TERMINATE,          // Marks end of one-side communication
    MALLET_POSITION,    // Contains mallet position data
    E_STOP,             // E-Stop triggered event

    COUNT               // Sentinal value
};

class Packet {
    private:
        std::vector<uint8_t>::iterator _read_iter;  // Read iterator for reading packet data
        std::vector<uint8_t> _data;                 // Byte data
        bool _finalized = false;                    // If the packet is finalized or not

    public:
        friend class SerialLink;

        /***
         * @brief Creates a packet with the desired action type
         * 
         * @param action    The desired action
         */
        Packet(Action action = Action::TERMINATE) {
            _data.resize(2);

            _data[0] = 2;                   // Length
            _data[1] = (uint8_t) action;    // Action
            
            resetRead();
        }

        /***
         * @brief Creates a packet from a vector of bytes
         * 
         * @param bytes         The vector of bytes
         * @param finalized     Whether the packet is ready to send or not, defaults to true
         */
        Packet(const std::vector<uint8_t>& bytes, bool finalized = true) {
            _finalized = finalized;
            _data = bytes;

            resetRead();
        }

        /***
         * @brief Resets the read iterator to the first byte of message data
         */
        void resetRead() {
            _read_iter = _data.begin() + 2;
        }

         /***
         * @brief Runs final calculations to prep packet for sending
         */
        void finalize() {
            // Ignore if already finalized
            if (_finalized) return;

            // CRC byte
            _data.push_back(0x00);

            // Adjust length parameter
            _data[0] = _data.size();
            
            // Calculate CRC
            uint8_t crc = 0;
            for (uint8_t byte : _data)
                crc += byte;

            // Write CRC
            _data[_data.size() - 1] = crc;

            // Mark as finalized
            _finalized = true;
        }

        /**
         * @brief Determines if the CRC in the packet matches with the data inside the packet
         * 
         * @return If the packet is valid
         */
        bool isValid() const {
            uint8_t calc_crc = 0;
            for (size_t i = 0; i < _data.size() - 1; i++)
                calc_crc += _data[i];
                
            return calc_crc == crc();
        }

        /**
         * @brief Returns the packet's action
         * 
         * @return The packet's action
         */
        Action action() const {
            return (Action) _data[1];
        }

        /**
         * @brief Returns the packet's total length
         * 
         * @return The packet's total length
         */
        uint8_t length() const {
            return _data[0];
        }

        /**
         * @brief Returns the packet's payload legth
         * 
         * @return The packet's payload length
         */
        uint8_t payloadLength() const { return length() - 3; }

        /**
         * @brief Returns the packet's stored CRC
         * 
         * @return The packet's stored CRC
         */
        uint8_t crc() const {
            return _data[_data.size() - 1];
        }

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