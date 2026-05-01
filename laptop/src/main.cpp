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
#include "Motion/Table.hpp"
#include "Interaction/RoutineButton.hpp"
#include "Interaction/GuiWindow.hpp"
#include "Types/Command.hpp"
#include "Types/Timer.hpp"
#include "Constants.hpp"

/*******************
  Gui Interactions
*******************/

GuiWindow window("GUI Window", {430, 260});

/*************************
  Command line arguments
*************************/

std::string com_port;

const std::unordered_map<std::string, Command> COMMAND_LIST = {
    {"track", Command({
        {"yellow",  []() { Table::setTracker<CameraTracker>(cv::Scalar{20, 110, 110}, cv::Scalar{40, 255, 255}); } },
        {"green",   []() { Table::setTracker<CameraTracker>(cv::Scalar{70, 50, 50}, cv::Scalar{100, 255, 255}); } },
        {"phys",    []() { Table::setTracker<PhysicsTracker>(); } },
        {"none",    []() { Table::setTracker<NoOperationTracker>(); } },
    })},

    {"com", Command({
        { "0",  []() { com_port = "\\\\.\\COM0"; } },
        { "1",  []() { com_port = "\\\\.\\COM1"; } },
        { "2",  []() { com_port = "\\\\.\\COM2"; } },
        { "3",  []() { com_port = "\\\\.\\COM3"; } },
        { "4",  []() { com_port = "\\\\.\\COM4"; } },
        { "5",  []() { com_port = "\\\\.\\COM5"; } },
        { "6",  []() { com_port = "\\\\.\\COM6"; } },
        { "7",  []() { com_port = "\\\\.\\COM7"; } },
        { "8",  []() { com_port = "\\\\.\\COM8"; } },
        { "9",  []() { com_port = "\\\\.\\COM9"; } },
        { "10", []() { com_port = "\\\\.\\COM10"; } },
        { "11", []() { com_port = "\\\\.\\COM11"; } },
        { "12", []() { com_port = "\\\\.\\COM12"; } },
    })},

    {"norender", Command({
        { "", []() { Table::hideRender(); }},
    })}
};

const std::unordered_map<std::string, std::string> DEFAULT_COMMANDS = {
    {"track", "yellow"},
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
    // Initialization logging
    Timer init_timer;
    Timer log_timer;

    auto log_msg = [](Timer& timer, const std::string& msg) {
        std::clog << std::setw(30) << std::left << (msg + ":");
        std::clog << std::setprecision(5)
            << 1e-3 * timer.delta<std::chrono::microseconds>()
            << " ms\n";
        timer.reset();
    };

    // Initialize the table
    Table::init();
    Table::setRoutine<NoOperationRoutine>();
    log_msg(log_timer, "Initialized table");

    // Run user commands
    const auto commands = parseUserCommands(argc, argv);
    for (auto& [command, argument] : commands) {
        // Find command in list of commands
        const auto result = COMMAND_LIST.find(command);
        if (result == COMMAND_LIST.end())
            throw(std::runtime_error("Unknown command: " + command));
        
        // Run the command with the provided argument
        result->second.run(argument);

        if (argument == "")
            log_msg(log_timer, "Ran command [" + command + "]");
        else
            log_msg(log_timer, "Ran command [" + command + "=" + argument + "]");
    }

    // Initialize serial comms
    SerialLink::init(com_port);
    log_msg(log_timer, "Initialized communications");

    // Initialize gui window
    window.addButton<RoutineButton<NoOperationRoutine>>(    {10, 10},   {200, 50}, {255, 255, 255}, "None");
    window.addButton<RoutineButton<ManualRoutine>>(         {220, 10},  {200, 50}, {255, 255, 255}, "Manual");
    window.addButton<RoutineButton<MotionTestRoutine>>(     {10, 70},   {200, 50}, {255, 255, 255}, "Motion Test");
    window.addButton<RoutineButton<DodgeRoutine>>(          {220, 70},  {200, 50}, {255, 255, 255}, "Dodge");
    window.addButton<RoutineButton<BasicDefenseRoutine>>(   {10, 130},  {200, 50}, {255, 255, 255}, "Defense 1");
    window.addButton<RoutineButton<AdvancedDefenseRoutine>>({220, 130}, {200, 50}, {255, 255, 255}, "Defense 2");
    window.addButton<RoutineButton<StrikeTestRoutine>>(     {10, 190},  {200, 50}, {255, 255, 255}, "Offense 1");
    window.addButton<RoutineButton<BasicOffenseRoutine>>(   {220, 190}, {200, 50}, {255, 255, 255}, "Offense 2");
    window.draw();
    log_msg(log_timer, "Initialized " + window.getName());

    // Initialization success
    log_msg(init_timer, "Initialization complete");
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

    while (true) {
        // OpenCV tracking + rendering
        Table::updateTracker();

        // Gui interface
        window.fetchUserInput();
    }

    return 0;
}
