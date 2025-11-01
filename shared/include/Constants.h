#pragma once

#include "Types/Point2.hpp"

namespace Constants {
    // Serial communication
    const uint32_t BAUD_RATE = 115200;              // Baud rate for USB serial communication
    const char COM_PORT[] = "COM5";                 // Desired COM port

    // Table
    const Point2<double> TABLE_SIZE(26.5, 54.0);    // Size of the table (inches)

    // Mallet
    const Point2<double> MALLET_HOME(13.25, 10.0);  // Homing location for the mallet
    const double MALLET_SPEED = 98.4251969;         // Estimated mallet speed (inch/sec)
    const double MALLET_RADIUS = 2.0;               // Radius in inches
    
    // Puck
    const Point2<double> PUCK_HOME(13.25, 50.0);    // Homing location for the puck (initialization)
    const double PUCK_SPEED = 137.795276;           // Estimated puck speed (inch/sec)
    const double PUCK_RADIUS = 1.25;                // Radius in inches

    // General
    const int64_t SAMPLE_RATE = 1e6 / 90;           // Camera sample rate (microseconds)
    const size_t NUM_SAMPLE_POINTS = 20;            // Number of sample points
}