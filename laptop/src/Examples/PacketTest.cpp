#include "Comms/Packet.hpp"
#include "Types/Point2.hpp"
#include <iostream>
#include <iomanip>

Point2<double> point(20.6, 10.5);

int main() {
    Packet packet(Action::MALLET_POSITION);
    packet << point;
    packet.finalize();

    packet.resetRead();
    std::cout << packet.read<Point2<double>>() << std::endl;

    packet.resetRead();
    std::cout << std::hex;
    for (uint8_t i = 0; i < packet.payloadLength(); i++)
        std::cout << (int) packet.read<uint8_t>() << " ";
    std::cout << std::endl;
    
    return 0;
}