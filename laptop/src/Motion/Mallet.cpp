#include "Motion/Mallet.h"
#include "Constants.h"

#include <algorithm>
#include <cmath>

Point2<double> Mallet::_target = Constants::Mallet::HOME;
MovingObject Mallet::_mallet;

bool compTimestamps(const Point3<double>& t1, const Point3<double>& t2) {
    return t1.z < t2.z;
}

// Constant velocity, infinite accelleration
double Mallet::timeToReach(const Point2<double>& pos) {
    return ((pos - _mallet.position()).magnitude()) / Constants::Mallet::SPEED;
}

bool Mallet::canReach(const Point2<double> pos) {
    // Ensure position is within reach
    if (pos.x < Constants::Mallet::LIMIT_BL.x || pos.y < Constants::Mallet::LIMIT_BL.y)
        return false;
    
    else if (pos.x > Constants::Mallet::LIMIT_TR.x || pos.y > Constants::Mallet::LIMIT_TR.y)
        return false;
    
    return true;
}

Point2<double> Mallet::chooseTarget(const std::vector<Point3<double>>& timestamps) {
    // Create copy of timestamps
    std::vector<Point3<double>> ts = timestamps;

    // Set time for each trajectory to be the mallet's time of arrival
    double best_weight = -std::numeric_limits<double>::infinity();
    Point2<double> best_target = Constants::Mallet::HOME;
    Point2<double> pos = _mallet.position();
    
    for (Point3<double>& t : ts) {
        Point2<double> tp(t.x, t.y);

        // Ignore if unreachable
        if (!canReach(tp))
            continue;

        // Parameters for choosing a target
        double reach_time = timeToReach(tp);

        double margin = t.z - reach_time;
        if (margin < 0)
            continue;

        double dist = (tp - pos).magnitude();

        // Weight targets by desire
        // weight points that are closer to mallet higher
        double weight = 
            - 0.25 * margin;                                // Relative time between arrivals
            // + 0.5 / (1 + sqrt(t.z))                           // Time the puck arrives
            // + 0.75 / (1 + dist);   // The distance from the puck's current location (normalized for time)

        // Check if this weight is the best
        if (weight > best_weight) {
            best_weight = weight;

            best_target = {
                std::clamp(tp.x, Constants::Mallet::LIMIT_BL.x + 1, Constants::Mallet::LIMIT_TR.x - 1),
                std::clamp(tp.y, Constants::Mallet::LIMIT_BL.y + 1, Constants::Mallet::LIMIT_TR.y - 1)
            };
        }
    }
    
    // Only modify target if outside of acceptable tolerance
    if ((_target - best_target).magnitude() > _TARGET_ERR)
        _target = best_target; //_target = 0.8 * _target + 0.2 * best_target;

    return _target;
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