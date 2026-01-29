/*
    Main executable for the Tracking and Control System

    This file, as well as its dependencies, is in charge of handling:
    - Tracking the position of the puck with a camera
    - Determine at what locations the robot can move it's mallet to
        - Running the necessary physics calculations to track where the puck will be in the future
        - Making an informed decision from these calculations as to where the mallet will move
    - Maintaining a constant communication channel with the microcontroller
        - Sending the current target location out to the microcontroller
        - Reading in the current location of the mallet
*/

/************
  Libraries
************/

#include "Simulation/Table.h"
#include "Comms/SerialLink.h"
#include "Motion/Mallet.h"
#include "Motion/Puck.h"

#include <algorithm>
#include <thread>
#include <mutex>


/********************
  Packet Management
********************/

void HANDLE_PACKET(Packet& packet) {
    Action action = packet.action();
    packet.resetRead();

    switch(action) {
        case Action::MALLET_POSITION: {
            Point2<double> p = packet.read<Point2<double>>();
            Mallet::moveTo(p);
            std::cout << p;
            break;
        }
            
        default:
            break;
    }
}


/***********************
  Asynchronous Threads
***********************/

// Camera image processing
void PUCK_TRACKING() {
    while (true) {
        Puck::locate();
    }
}

// Communication with the microcontroller
void RECEIVE_PACKETS() {
    while (true) {
        Packet packet = SerialLink::read();
        HANDLE_PACKET(packet);
    }
}

// Mallet control
void MALLET_CONTROL() {
    while (true) {
        // Get puck trajectory
        std::vector<Point3<double>> timestamps = Puck::estimateTrajectory();

        // Get mallet's target location
        Point2<double> target = Mallet::chooseTarget(timestamps);

        // Send target out to gantry
        Packet packet(Action::MALLET_POSITION);
        packet << target;

        SerialLink::send(packet);
    }
}


/*****************
  Initialization
*****************/

bool INIT_MAIN() {
    try {
        Puck::initTracking();   // Initialize the puck tracking
        SerialLink::init();     // Initialize serial comms
    }

    catch(const std::exception& e) {
        std::cerr << e.what() << '\n';
        return false;
    }

    return true;
}


/****************
  Main Function
****************/

int main() {
    // Initialize
    if (!INIT_MAIN()) return -1;

    // Create threads
    std::thread receive_packets(RECEIVE_PACKETS);
    // std::thread mallet_control(MALLET_CONTROL);
    // std::thread puck_tracking(PUCK_TRACKING);

    // Run threads async
    receive_packets.detach();
    // mallet_control.detach();
    // puck_tracking.detach();
    
    // Yield main
    while (true) {
        Table::render();
    }

    return 0;
}