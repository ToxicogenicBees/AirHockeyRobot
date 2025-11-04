#pragma once

#include "Types/Point2.hpp"

namespace Constants {
    // Serial communication
    constexpr uint32_t BAUD_RATE = 115200;              // Baud rate for USB serial communication
    constexpr char COM_PORT[] = "COM5";                 // Desired COM port

    // Table
    const Point2<double> TABLE_SIZE(26.5, 54.0);        // Size of the table (inches)

    // Mallet
    const Point2<double> MALLET_HOME(13.25, 10.0);      // Homing location for the mallet
    constexpr double MALLET_SPEED = 98.4251969;         // Estimated mallet speed (inch/sec)
    constexpr double MALLET_RADIUS = 2.0;               // Radius in inches

    const Point2<double> MALLET_LIMIT_BL                // Table range from origin the mallet can reach
        = {MALLET_RADIUS, MALLET_RADIUS};   
    const Point2<double> MALLET_LIMIT_TR
        = {TABLE_SIZE.x - MALLET_RADIUS, 20.0};
    
    // Puck
    const Point2<double> PUCK_HOME(13.25, 50.0);        // Homing location for the puck (initialization)
    constexpr double PUCK_SPEED = 137.795276;           // Estimated puck speed (inch/sec)
    constexpr double PUCK_RADIUS = 1.25;                // Radius in inches

    // General
    constexpr int64_t SAMPLE_RATE = 1e6 / 90;           // Camera sample rate (microseconds)
    constexpr size_t NUM_SAMPLE_POINTS = 20;            // Number of sample points
}