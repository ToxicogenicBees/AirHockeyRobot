#include "Motion/Mallet.h"
#include "Constants.h"

#include <algorithm>
#include <cmath>

MovingObject Mallet::_mallet;

bool compTimestamps(const Point3<double>& t1, const Point3<double>& t2) {
    return t1.z < t2.z;
}

double Mallet::_squaredTimeToReach(const Point2<double>& pos) {
    return (pos - _mallet.position()).squaredMagnitude() / (Constants::MALLET_SPEED * Constants::MALLET_SPEED);
}

// Constant velocity, infinite accelleration
double Mallet::timeToReach(const Point2<double>& pos) {
    return std::sqrt(_squaredTimeToReach(pos));
}

bool Mallet::canReach(const Point3<double>& timestamp) {
    return _squaredTimeToReach({timestamp.x, timestamp.y}) <= timestamp.z * timestamp.z;
}

Point2<double> Mallet::chooseTarget(const Matrix<Point3<double>>& timestamps) {
    // Create copy of timestamps
    auto ts = timestamps;

    // Set time for each trajectory to be the mallet's time of arrival
    for (auto& t : ts)
        t.z = (t.z * t.z) - _squaredTimeToReach({t.x, t.y});

    // Sort by time
    std::sort(ts.begin(), ts.end(), compTimestamps);

    // Choose first non-zero time
    for (auto t : ts)
        if (t.z >= 0) return {t.x, t.y};
    
    // If mallet can't reach any of these, go to home
    return Constants::MALLET_HOME;
}

void Mallet::moveTo(const Point2<double>& new_pos, int64_t micsec) {
    _mallet.moveTo(new_pos, micsec);
}

void Mallet::orient(const Point2<double>& pos, const Point2<double>& vel) {
    _mallet.orient(pos, vel);
}

Point2<double> Mallet::position() {
    return _mallet.position();
}

Point2<double> Mallet::velocity() {
    return _mallet.velocity();
}