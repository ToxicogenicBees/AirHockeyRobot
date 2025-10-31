#pragma once

#include "Packet.h"

/* 

This file sets up the general USB communication scheme between
the microcontroller and the laptop. This file provides a
consistant interface between the two. Both systems define their
own version of this file separately.

*/


class SerialLink {
    private:

    public:
        void send(Packet& packet);

        bool readByte(Packet& packet);
};