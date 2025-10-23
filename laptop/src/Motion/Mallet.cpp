#include "../../include/Motion/Mallet.h"
#include <cmath>

// Constant velocity, infinite accelleration
double Mallet::timeToReach(const Point2<double>& pos) {
    // Distance to point
    double dist = (pos - _pos).magnitude();

    // Return time to reach
    return dist / Constants::MALLET_SPEED;
}

bool Mallet::canReach(const Point3<double>& timestamp) {
    return timeToReach({timestamp.x, timestamp.y}) <= timestamp.z;
}