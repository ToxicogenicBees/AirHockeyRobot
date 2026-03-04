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

#include "Types/VelocityProfile.hpp"
#include "Comms/SerialLink.hpp"
#include "Simulation/Table.h"
#include "State/StateTracker.h"
#include "State/KeyLog.h"
#include "Motion/PuckTracker.h"
#include "Motion/Mallet.h"
#include "Motion/Puck.h"
#include "Constants.h"

#include <windows.h>

#include <algorithm>
#include <iostream>
#include <thread>
#include <mutex>

PuckTracker puck_tracker(0, cv::CAP_ANY);

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
            // std::clog << p << "\n";
            break;
        }

        case Action::LimitSwitches: {
            uint8_t pressed = packet.read<uint8_t>();

            // check for which limit switches pressed
            if (pressed & Constants::LimitSwitch::LEFT_PRESSED) {
                // left was pressed, decide what movements invalid
            }
            if (pressed & Constants::LimitSwitch::RIGHT_PRESSED) {
                // right was pressed, decide what movements invalid
            }
            if (pressed & Constants::LimitSwitch::BOTTOM_PRESSED) {
                // bottom was pressed, decide what movements invalid
            }
            if (pressed & Constants::LimitSwitch::TOP_PRESSED) {
                // top was pressed, decide what movements invalid
            }

            break;
        }

        case Action::DistanceSensorRead: {
            std::clog << "Updated position with distance sensors.\n";

            break;
        }

        case Action::MalletHome: {
            std::clog << "Bad distance sensor read.\n";

            break;
        }
            
        default:
            break;
    }
}


/***********************
  Asynchronous Threads
***********************/

// Communication with the microcontroller
void RECEIVE_PACKETS() {
    // Laptop initializes communication
    SerialLink::process(true);

    while (true) {
        SerialLink::process();
    }
}

// Mallet control
void MALLET_CONTROL() {
    // get initial position with distance sensors
    Packet packet(Action::DistanceSensorRead);
    SerialLink::buffer(packet);

    Sleep(2000);

    // Packet packet(Action::DistanceSensorRead);
    SerialLink::buffer(packet);

    Sleep(2000);

    while (true) {
        // Send target location
        auto trajectory = Puck::estimateTrajectory();
        auto target = Mallet::chooseTarget(trajectory);

        // vary speed based on how far away target is
        // send velocity profile settings
        double dist_mag = (Mallet::position() - target).magnitude();
        if (dist_mag < 1) {
            VelocityProfile profile(0, 0, 50, 50);
            Packet vel_packet(Action::VelocityProfile);
            vel_packet << profile;
            SerialLink::buffer(vel_packet);
        } else if (dist_mag < 2) {
            VelocityProfile profile(0, 0, 150, 150);
            Packet vel_packet(Action::VelocityProfile);
            vel_packet << profile;
            SerialLink::buffer(vel_packet);
        } else if (dist_mag < 6) {
            VelocityProfile profile(0, 0, 350, 350);
            Packet vel_packet(Action::VelocityProfile);
            vel_packet << profile;
            SerialLink::buffer(vel_packet);
        } else if (dist_mag < 15) {
            VelocityProfile profile(0, 0, 600, 600);
            Packet vel_packet(Action::VelocityProfile);
            vel_packet << profile;
            SerialLink::buffer(vel_packet);
        }
        


        Packet pos_packet(Action::MalletPosition);
        Point2<double> targetPosition = (25.4 * (target - Constants::Mallet::LIMIT_BL));
        pos_packet << targetPosition;
        SerialLink::buffer(pos_packet);

        // if already made it to the target point, then take
        // distance sensor reading of mallet location
        Point2<double> temp = (Mallet::position()-Constants::Mallet::LIMIT_BL)*25.4;  // in mm
        if (abs(temp.x - targetPosition.x) < 5 && abs(temp.y - targetPosition.y) < 5) {
            Packet packet(Action::DistanceSensorRead);
            SerialLink::buffer(packet);
        }
    }
}


/*****************
  Initialization
*****************/

bool INIT_MAIN() {
    try {
        // Initialize state tracker
        StateTracker::init();
        std::clog << "Initialized state tracker\n";

        // Initialize puck tracker
        puck_tracker.init();
        std::clog << "Initialized puck tracker\n";

        // Initialize serial comms
        SerialLink::init(HANDLE_PACKET);
        std::clog << "Initialized serial link on " << Constants::Comms::COM_PORT << '\n';

        // End of initialization
        std::clog << '\n';
    }

    // Handle errors gracefully
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
    if (!INIT_MAIN())
        return 1;

    // Create threads
    std::thread receive_packets(RECEIVE_PACKETS);
    std::thread mallet_control(MALLET_CONTROL);

    // Run threads async
    receive_packets.detach();
    mallet_control.detach();
    
    // Yield main
    while (1) {
        // Puck tracking
        puck_tracker.captureFrame();

        // Visualizers
        puck_tracker.displayFrame();
        Table::render();
    }

    return 0;
}
