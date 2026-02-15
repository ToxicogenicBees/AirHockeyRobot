#include <opencv2/opencv.hpp>
#include <thread>

#include "Comms/WinUSBLink.hpp"
#include "Simulation/Table.h"
#include "Motion/Mallet.h"
#include "Motion/Puck.h"
#include "Constants.h"

const double MAX_X_MALLET = 587;
const double MAX_Y_MALLET = 505;

const double X_OFFSET = 34.7;
const double Y_OFFSET = 37.35;

void HANDLE_PACKET(Packet& packet) {
    Action action = packet.action();
    packet.resetRead();

    switch(action) {
        case Action::MALLET_POSITION: {
            Point2<double> p = packet.read<Point2<double>>();
            p.x = ((p.x - X_OFFSET) / MAX_X_MALLET) * Constants::Table::SIZE.x;
            p.y = ((p.y - Y_OFFSET) / MAX_Y_MALLET) * Constants::Table::SIZE.y/2;
            Mallet::moveTo(p); // moveTo is concurrent safe
            // std::cout << p;
            break;
        }
            
        default:
            break;
    }
}

// Communication with the microcontroller
void RECEIVE_PACKETS() {
    WinUSBLink::process();
}

bool INIT_MAIN() {
    try {
        WinUSBLink::init(HANDLE_PACKET);     // Initialize serial comms
    }

    catch(const std::exception& e) {
        std::cerr << e.what() << '\n';
        return false;
    }

    return true;
}

int main() {
    // Initialize
    if (!INIT_MAIN()) return -1;

    // Initialize the table
    Puck::orient(Constants::Puck::HOME, Constants::Puck::SPEED * Point2<double>(0.5 * std::sqrt(2), 0.5 * std::sqrt(2)));
    Mallet::orient(Constants::Mallet::HOME);

    // Serial read thread
    std::thread receive_packets(RECEIVE_PACKETS);
    receive_packets.detach();

    // Render processing
    while (true) {
        Table::render();
    }

    return 0;
}