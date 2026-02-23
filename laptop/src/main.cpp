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

#include "Comms/SerialLink.hpp"
#include "Simulation/Table.h"
#include "State/StateTracker.h"
#include "State/KeyLog.h"
#include "Motion/Mallet.h"
#include "Motion/Puck.h"
#include "Constants.h"

#include <windows.h>

#include <algorithm>
#include <iostream>
#include <thread>
#include <mutex>


/********************
  Packet Management
********************/

void HANDLE_PACKET(Packet& packet) {
    Action action = packet.action();
    packet.resetRead();

    switch(action) {
        case Action::MalletPosition: {
            Point2<double> p = packet.read<Point2<double>>();
            p /= 25.4; // mm to inches
            p += Constants::Mallet::LIMIT_BL;

            Mallet::moveTo(p);
            std::clog << p << "\n";
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
bool firstSend = true;
void RECEIVE_PACKETS() {
    while (true) {
        SerialLink::process(firstSend);
        firstSend = false;
    }
}

// Mallet control
void MALLET_CONTROL() {
    Timer timer;

    while (true) {
        // send velocity profile settings
        Packet velPacket(Action::VelocityProfile);
        velPacket << uint16_t(15);  // min_rpm
        velPacket << uint16_t(15);  // max_rpm
        velPacket << uint8_t(25);  // accel_percent
        velPacket << uint8_t(10);  // decel_percent
        SerialLink::buffer(velPacket);

        // // Get puck trajectory
        // std::vector<Point3<double>> timestamps = Puck::estimateTrajectory();

        // // Get mallet's target location
        // Point2<double> target = Mallet::chooseTarget(timestamps);

        // Send target out to gantry
        Packet packet(Action::MalletPosition);
        packet << Point2<double>{150, 150};
        SerialLink::buffer(packet);

        // std::clog << "Sending Mallet Commands\n";
    }
}


/*****************
  Initialization
*****************/

bool INIT_MAIN() {
    try {
        // StateTracker::init();               // Initialize state tracker
        // std::clog << "Initialized state tracker\n";

        // Puck::initTracking();               // Initialize the puck tracking
        // std::clog << "Initialized puck tracker\n";

        SerialLink::init(HANDLE_PACKET);    // Initialize serial comms
        std::clog << "Initialized serial link on " << Constants::Comms::COM_PORT << '\n';

        // End of initialization
        std::clog << '\n';
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
    if (!INIT_MAIN()) return 1;

    // Create threads
    std::thread receive_packets(RECEIVE_PACKETS);
    std::thread mallet_control(MALLET_CONTROL);
    // std::thread puck_tracking(PUCK_TRACKING);

    // Run threads async
    receive_packets.detach();
    mallet_control.detach();
    // puck_tracking.detach();
    
    // Yield main
    while (1) {
        Table::render();
    }

    return 0;
}