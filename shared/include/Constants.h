#pragma once

#include "Types/Point2.hpp"

namespace Constants {
    namespace Comms {
        constexpr uint32_t BAUD_RATE = 115200;          // Baud rate for USB serial communication
        constexpr char COM_PORT[] = "COM5";             // Desired COM port
    }
    
    namespace Table {
        const Point2<double> SIZE(26.5, 54.0);          // Size of the table (inches)
        const double FRICTION = 0;                      // Coefficient of friction of the surface of the table
    }

    namespace Mallet {
        const Point2<double> HOME = {13.25, 10.0};      // Homing location for the mallet
        constexpr double SPEED = 98.4251969;            // Estimated mallet speed (inch/sec)
        constexpr double RADIUS = 2.0;                  // Radius in inches

        const Point2<double> LIMIT_BL                   // Bottom-left limit of mallet motion
            = {RADIUS, RADIUS};
        const Point2<double> LIMIT_TR                   // Top-right limit of mallet motion
            = {Table::SIZE.x - RADIUS, 20.0};
    }
    
    namespace Puck {
        const Point2<double> HOME = {13.25, 50.0};      // Homing location for the puck (initialization)
        constexpr double SPEED = 137.795276;            // Estimated puck speed (inch/sec)
        constexpr double RADIUS = 1.25;                 // Radius in inches
    }
    
    // General
    constexpr int64_t SAMPLE_RATE = 1e6 / 90;           // Camera sample rate (microseconds)
    constexpr size_t NUM_SAMPLE_POINTS = 20;            // Number of sample points
}