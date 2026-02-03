#pragma once

#include "Comms/Packet.hpp"

#ifdef WIN32_LEAN_AND_MEAN
    #include <windows.h>
#endif

class SerialLink {
    private:
        #ifdef WIN32_LEAN_AND_MEAN
            static HANDLE _h_serial;
        #endif

        static bool _ready;

    public:
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