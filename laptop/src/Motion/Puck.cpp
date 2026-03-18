#include <opencv2/opencv.hpp>
#include <algorithm>

#include "Motion/Mallet.h"
#include "Motion/Puck.h"

MovingObject Puck::_puck;

std::vector<Puck::Timestamp> Puck::estimateTrajectory(bool ignore_return) {
    Point2<double> pos = _puck.position();
    Point2<double> vel = _puck.velocity();

    // If the puck isn't moving, return early
    if (vel.magnitude() < 1e-8)
        return {};

    // If the puck is moving away and it should be ignored, return early
    if (vel.y > 0 && ignore_return)
        return {};

    // Determine how long the puck will move for
    double time_of_arrival = (vel.y > 0 ? 2 * (Constants::Table::SIZE.y - Constants::Puck::RADIUS) - pos.y : Constants::Puck::RADIUS - pos.y) / vel.y;

    // Determine number of sample points
    const size_t NUM_SAMPLES = std::min(
        (size_t) (time_of_arrival / (1e-6 * Constants::SAMPLE_PERIOD)),
        Constants::MAX_SAMPLE_POINTS
    );
    
    double time_step = time_of_arrival / NUM_SAMPLES;
    
    // Calculate trajectory
    std::vector<Timestamp> trajectory;
    for (size_t i = 0; i < NUM_SAMPLES; i++) {
        double time = i * time_step;
        auto orientation = determineFutureOrientation(time);
        trajectory.push_back({orientation, time});
    }
    
    // Return trajectory
    return trajectory;
}

Ray2<double> Puck::determineFutureOrientation(double dt) {
    Point2<double> pos = _puck.position();
    Point2<double> vel = _puck.velocity();

    // Exit early if not moving
    if (vel.squaredMagnitude() <= Constants::FP_ERR)
        return {pos};

    // Raw displacement
    Point2<double> new_pos = pos + dt * vel;

    // Effective puck bounds (accounting for the radius of the puck)
    const Point2<double> MIN = Constants::Puck::RADIUS * Point2<double>::one();
    const Point2<double> MAX = Constants::Table::SIZE - MIN;

    // Reflect off an axis
    auto reflect = [](double& new_pos, double& vel, double min, double max) {
        double range = max - min;
        double period = 2 * range;
        double mod = std::fmod(new_pos - min, period);

        if (mod < 0)
            mod += period;
        
        if (mod < range)
            new_pos = min + mod;
        else {
            new_pos = max - (mod - range);
            vel *= -1;
        }
    };

    // Reflect off each axis
    reflect(new_pos.x, vel.x, MIN.x, MAX.x);
    reflect(new_pos.y, vel.y, MIN.y, MAX.y);

    return { new_pos , vel };
}

Point2<double> Puck::reflectedVelocity() {
    // v(T+) = v(T-) + (1 + a_r)(nn^t)(V - b(T-))
    // v(T+) = (I - (1 + a_r)nn^T)v(T-)

    // Get the normal to the collision
    Point2<double> n = (position() - Mallet::position()).normal();
    Point2<double> v = velocity();

    // Return new velocity
    return v + (1 + Constants::Table::COEF_REST) * n.dot(Mallet::velocity() - v) * n;
}

void Puck::moveTo(const Point2<double>& new_pos, int64_t micsec) {
    _puck.moveTo(new_pos, micsec);
}

void Puck::orient(const Ray2<double>& orientation) {
    _puck.orient(orientation);
}

Ray2<double> Puck::orientation() {
    return _puck.orientation();
}

Point2<double> Puck::position() {
    return _puck.position();
}

Point2<double> Puck::velocity() {
    return _puck.velocity();
}