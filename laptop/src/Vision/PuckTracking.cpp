#include "Vision/PuckTracking.h"

#include <opencv2/opencv.hpp>

bool PuckTracking::_initialized = false;
Puck PuckTracking::_puck;

void PuckTracking::moveTo(const Point2<double>& new_pos) {
    _puck.moveTo(new_pos);
}

void PuckTracking::init() {
    // Run initialization logic
    // ...

    // Toggle initializer flag
    _initialized = true;
}

void PuckTracking::locate() {
    // Take image of table

    // Scan image for puck

    // Adjust coordinates to inches
    Point2<double> inches = Constants::PUCK_HOME;

    // Update puck location
    _puck.moveTo(inches);
}

Matrix<Point3<double>> PuckTracking::estimateTrajectory() {
    return _puck.estimateTrajectory();
}