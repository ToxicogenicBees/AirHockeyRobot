#include "../../include/Motion/Mallet.h"
#include <cmath>

// Constant velocity, infinite accelleration
double Mallet::timeToReach(const Point2<double>& pos) {
    return (pos - _pos).magnitude() / Constants::MALLET_SPEED;
}

bool Mallet::canReach(const Point3<double>& timestamp) {
    return timeToReach({timestamp.x, timestamp.y}) <= timestamp.z;
}