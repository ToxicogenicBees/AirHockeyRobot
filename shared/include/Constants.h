#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "Types/Point2.hpp"

namespace Constants {
    namespace Comms {
        constexpr uint32_t BAUD_RATE = 115200;          // Baud rate for USB serial communication
        constexpr char COM_PORT[] = "\\\\.\\COM4";      // Desired COM port
        constexpr int64_t TIMEOUT = 10;                 // Timeout in seconds
    }
    
    namespace Table {
        const Point2<double> HUMAN_GOAL(13.25, 54.0);   // Center of human's goal (inches)
        const Point2<double> ROBOT_GOAL(13.25, 0.0);    // Center of robot's goal (inches)
        const Point2<double> SIZE(26.5, 54.0);          // Size of the table (inches)
        
        constexpr double GOAL_WIDTH = 9.0;              // Width of goals (inches)
        constexpr double COEF_FRIC = 0;                 // Coefficient of friction for the surface of the table
        constexpr double COEF_REST = 1;                 // Coefficient of friction of the walls of the table
    }

    namespace Mallet {
        const Point2<double> HOME = {13.25, 10.0};      // Homing location for the mallet
        constexpr double SPEED = 98.4251969;            // Estimated mallet speed (inch/sec)
        constexpr double ACCEL = 1181.10236;            // Estimated mallet accelleration (inch/sec^2)
        constexpr double RADIUS = 2.0;                  // Radius in inches

        const Point2<double> LIMIT_BL                   // Bottom-left limit of mallet motion
            = {1.59375, 3.0};
        const Point2<double> LIMIT_TR                   // Top-right limit of mallet motion
            = {Table::SIZE.x - 2.5, 25.0};
    }
    
    namespace Puck {
        const Point2<double> HOME = {13.25, 50.0};      // Homing location for the puck (initialization)
        constexpr double SPEED = 137.795276;            // Estimated puck speed (inch/sec)
        constexpr double RADIUS = 1.25;                 // Radius in inches
    }

    namespace LimitSwitch {
        const uint8_t LEFT_PRESSED = 0b00000001;
        const uint8_t RIGHT_PRESSED = 0b00000010;
        const uint8_t BOTTOM_PRESSED = 0b00000100;
        const uint8_t TOP_PRESSED = 0b00001000;
    }
    
    // General
    constexpr int64_t SAMPLE_PERIOD = 1e6 / 100;        // Camera sample period (microseconds)
    constexpr size_t MAX_SAMPLE_POINTS = 50;            // Maximum number of sample points
    constexpr double FP_ERR = 1e-8;                     // Consistant FP error bias
}

#endif
