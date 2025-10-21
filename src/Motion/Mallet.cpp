#include "../../include/Motion/Mallet.h"
#include <cmath>

void Mallet::readPosition(const Point2<double>& new_pos) {
    _cur_pos = new_pos;
}

double Mallet::timeToReach(const Point2<double>& pos) {
    // Distance to point
    double dist = (pos - _cur_pos).magnitude();

    // Return time to reach
    return dist / Constants::MAX_MALLET_SPEED;
}