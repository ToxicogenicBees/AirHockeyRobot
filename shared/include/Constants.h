#pragma once

#include "Types/Point2.hpp"

namespace Constants {
    // Table
    const Point2<double> TABLE_SIZE(26.5, 54.0);    // Size of the table (inches)

    // Mallet
    const double MALLET_SPEED = 98.4251969;         // Estimated mallet speed (inch/sec)
    const double MALLET_RADIUS = 2.0;               // Radius in inches
    
    // Puck
    const double PUCK_SPEED = 137.795276;           // Estimated puck speed (inch/sec)
    const double PUCK_RADIUS = 1.25;                // Radius in inches

    // General
    const double SAMPLE_RATE = 1.0 / 90;            // Camera sample rate
}