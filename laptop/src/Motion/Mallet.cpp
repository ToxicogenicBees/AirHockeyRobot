#include "Motion/Mallet.h"
#include "Constants.h"

#include <algorithm>
#include <cmath>

Point2<double> Mallet::_prev_target = Constants::Mallet::HOME;
MovingObject Mallet::_mallet;

bool compTimestamps(const Point3<double>& t1, const Point3<double>& t2) {
    return t1.z < t2.z;
}

// Constant velocity, infinite accelleration
double Mallet::timeToReach(const Point2<double>& pos) {
    return (pos - _mallet.position()).magnitude() / Constants::Mallet::SPEED;
}

bool Mallet::canReach(const Point2<double> pos) {
    // Ensure position is within reach
    if (pos.x < Constants::Mallet::LIMIT_BL.x || pos.y < Constants::Mallet::LIMIT_BL.y)
        return false;
    
    else if (pos.x > Constants::Mallet::LIMIT_TR.x || pos.y > Constants::Mallet::LIMIT_TR.y)
        return false;
    
    return true;
}

Point2<double> Mallet::chooseTarget(const Matrix<Point3<double>>& timestamps) {
    // Create copy of timestamps
    auto ts = timestamps;

    // Set time for each trajectory to be the mallet's time of arrival
    for (auto& t : ts)
        t.z -= timeToReach({t.x, t.y});

    // Sort by time
    std::sort(ts.begin(), ts.end(), compTimestamps);

    // Choose first time the mallet can reach (or home if none exist)
    Point2<double> target = Constants::Mallet::HOME;
    for (auto t : ts) {
        if (t.z > 0 && t.y <= Constants::Mallet::LIMIT_TR.y) {
            target = {t.x, t.y};
            break;
        }
    }

    // Only modify target if outside of acceptable tolerance
    if ((_prev_target - target).magnitude() > _TARGET_ERR)
        _prev_target = target;

    _prev_target.x = std::clamp(_prev_target.x, Constants::Mallet::LIMIT_BL.x, Constants::Mallet::LIMIT_TR.x);
    _prev_target.y = std::clamp(_prev_target.y, Constants::Mallet::LIMIT_BL.y, Constants::Mallet::LIMIT_TR.y);
        
    std::cout << _prev_target << std::endl;
    
    return _prev_target;
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