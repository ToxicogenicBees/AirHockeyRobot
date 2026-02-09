#ifndef SERIAL_LINK_HPP
#define SERIAL_LINK_HPP

#include "Comms/PacketBuffer.hpp"
#include "Comms/Packet.hpp"

#ifdef WIN32_LEAN_AND_MEAN
    #include <windows.h>
#endif

class SerialLink {
    private:
        #ifdef WIN32_LEAN_AND_MEAN
            static HANDLE _h_serial;
        #endif

        static PacketBuffer _receive_buffer;
        static PacketBuffer _send_buffer;
        static bool _ready;

        // Send a finalized packet over the link
        static void _sendOverLink(const Packet& packet);

    public:
        /**
         * @brief   Initializes the communication link on the device's end
         */
        static void init();

        /**
         * @brief   Buffers a packet into the packet buffer
         * 
         * @param   packet  The packet being buffered
         */
        static void buffer(const Packet& packet);
        
        /**
         * @brief   Sends all buffered packets over the link
         */
        static void send();

        /**
         * @brief   Reads a packet from the link
         */
        static Packet read();
};

#endif