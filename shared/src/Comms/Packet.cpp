#include "../../include/Comms/Packet.h"

#include <cstring>

Packet::Packet(uint8_t action) {
    _data.resize(2);

    _data[0] = action;  // Action
    _data[1] = 2;       // Length
    
    resetRead();
}

void Packet::resetRead() {
    _read_iter = _data.begin() + 2;
}

bool Packet::isValid() {
    uint8_t calc_crc = 0;
    for (size_t i = 0; i < _data.size() - 2; i++)
        calc_crc += _data[i];
        
    return calc_crc == crc();
}

void Packet::finalize() {
    // CRC byte
    _data.push_back(0x00);

    // Adjust length parameter
    _data[1] = _data.size();
    
    // Calculate CRC
    uint8_t crc = 0;
    for (uint8_t byte : _data)
        crc += byte;

    // Write CRC
    _data[_data.size() - 1] = crc;
}