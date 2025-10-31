#include "../../../shared/include/Comms/Packet.h"
#include <iostream>

Packet packet;

int main() {
    Packet packet(Action::MALLET_POSITION);
    packet << 20.6 << 10.5;
    packet.finalize();

    packet.resetRead();

    std::cout << packet.read<double>() << std::endl;
    std::cout << packet.read<double>() << std::endl;
    
    return 0;
}