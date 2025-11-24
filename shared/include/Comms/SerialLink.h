#pragma once

#include "Comms/Packet.hpp"

#ifdef WIN32_LEAN_AND_MEAN
    #include <windows.h>
#endif

struct SerialLink {
    #ifdef WIN32_LEAN_AND_MEAN
        static HANDLE hSerial;

        static void readWaitCommLoop(std::vector<Packet>*);
    #endif

    /**
     * @brief   Initializes the communication link on the device's end
     */
    static void init();
    
    /**
     * @brief   Sends a packet over the link
     */
    static void send(Packet& packet);

    /**
     * @brief   Reads a packet from the link
     */
    static Packet read();
};