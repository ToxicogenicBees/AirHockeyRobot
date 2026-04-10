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

#include <windows.h>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <unordered_map>
#include <vector>
#include <functional>
#include <string>
#include <thread>

#include "Routines/MotionTestRoutine.hpp"
#include "Routines/BasicDefenseRoutine.hpp"
#include "Routines/StrikeTestRoutine.hpp"
#include "Routines/DodgeRoutine.hpp"
#include "Routines/AdvancedDefenseRoutine.hpp"
#include "Routines/BasicOffenseRoutine.hpp"
#include "Routines/NoOperationRoutine.hpp"
#include "Routines/ManualRoutine.hpp"
#include "Tracking/CameraTracker.hpp"
#include "Tracking/PhysicsTracker.hpp"
#include "Tracking/NoOperationTracker.hpp"
#include "Comms/SerialLink.hpp"
#include "Types/Command.hpp"
#include "Motion/Table.hpp"
#include "Types/Timer.hpp"
#include "Constants.hpp"

/*************************
  Command line arguments
*************************/

const std::unordered_map<std::string, Command> COMMAND_LIST = {
    {"track", Command({
        {"yellow",  []() { Table::setTracker<CameraTracker>(cv::Scalar{20, 100, 100}, cv::Scalar{40, 255, 255}); } },
        {"green",   []() { Table::setTracker<CameraTracker>(cv::Scalar{70, 50, 50}, cv::Scalar{100, 255, 255}); } },
        {"phys",    []() { Table::setTracker<PhysicsTracker>(); } },
        {"none",    []() { Table::setTracker<NoOperationTracker>(); } },
    })},

    {"diff", Command({
        { "0",      []() { Table::setRoutine<MotionTestRoutine>(); } },
        { "1",      []() { Table::setRoutine<DodgeRoutine>(); } },
        { "2",      []() { Table::setRoutine<BasicDefenseRoutine>(); } },
        { "3",      []() { Table::setRoutine<StrikeTestRoutine>(); } },
        { "4",      []() { Table::setRoutine<AdvancedDefenseRoutine>(); } },
        { "5",      []() { Table::setRoutine<BasicOffenseRoutine>(); } },
        { "manual", []() { Table::setRoutine<ManualRoutine>(); } },
        { "none",   []() { Table::setRoutine<NoOperationRoutine>(); } }
    })},

    {"norender", Command({
        { "", []() { Table::hideRender(); }}
    })}
};

const std::unordered_map<std::string, std::string> DEFAULT_COMMANDS = {
    {"track", "yellow"},
    {"diff", "0"},
};

/*****************
  Initialization
*****************/

std::unordered_map<std::string, std::string> parseUserCommands(int argc, char** argv) {
    // Parse commands list
    const std::vector<std::string> arguments(argv + 1, argv + argc);
    auto commands = DEFAULT_COMMANDS;

    for (auto it = arguments.begin(); it != arguments.end(); ++it) {
        std::string token = *it;
        std::string command;
        std::string value;

        // Handle --key=value
        auto eq_pos = token.find('=');
        if (eq_pos != std::string::npos) {
            command = token.substr(0, eq_pos);
            value = token.substr(eq_pos + 1);
        }
        else {
            command = token;

            // Check next argument
            auto next = std::next(it);
            if (next != arguments.end() && next->rfind("--", 0) != 0) {
                value = *next;
                ++it;
            }
            else {
                value = "";
            }
        }

        // Validate and add command
        if (COMMAND_LIST.find(command) == COMMAND_LIST.end())
            throw std::runtime_error("Unknown command: " + command);
        commands[command] = value;
    }

    return commands;
}

void initialize(int argc, char** argv) {
    // Start initialization
    Timer init_timer;

    // Initialize the table
    Table::init();

    // Initialize serial comms
    SerialLink::init();

    // Run user commands
    const auto commands = parseUserCommands(argc, argv);
    for (auto& [command, argument] : commands) {
        // Find command in list of commands
        const auto result = COMMAND_LIST.find(command);
        if (result == COMMAND_LIST.end())
            throw(std::runtime_error("Unknown command: " + command));
        
        // Run the command with the provided argument
        result->second.run(argument);
    }

    // Initialization success
    std::clog << "Initialized: " << std::setprecision(5)
        << 1e-6 * init_timer.delta<std::chrono::microseconds>()
        << " seconds\n";
}

/****************
  Main Function
****************/

int main(int argc, char** argv) {

    // Initialize main
    try {
        initialize(argc, argv);
    } catch(const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }

    // Ping packet processing
    SerialLink::registerHandler(Action::Ping, [](Packet& packet) {
        std::clog << "Pong!\n";
    });

    // Asynchronous packet processing
    std::thread receive_packets([]() {
        // Laptop initializes communication
        SerialLink::process(true);
        while (true) {
            SerialLink::process();
        }
    });
    receive_packets.detach();
    
    // Mallet tracking + controlling
    std::thread mallet_control([]() {
        while(true) {
            Table::updateRoutine();
        }
    });
    mallet_control.detach();

    // OpenCV tracking + rendering
    while (true) {
        Table::updateTracker();
    }

    return 0;
}
