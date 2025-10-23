#pragma once

#include <stdint.h>

class Packet {
    private:
        uint8_t* _data;     // Null-terminated string of bytes

    public:
        Packet() = default;
        ~Packet() = default;
};