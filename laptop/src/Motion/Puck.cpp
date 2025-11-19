#include <opencv2/opencv.hpp>
#include <algorithm>

#include "Motion/Mallet.h"
#include "Motion/Puck.h"


bool Puck::_initialized = false;
MovingObject Puck::_puck;

void Puck::initTracking() {
    // Run initialization logic
    // ...

    // Toggle initializer flag
    _initialized = true;
}

void Puck::locate() {
    // Take image of table

    // Scan image for puck

    // Adjust coordinates to inches
    Point2<double> inches = Constants::Puck::HOME;

    // Update puck location
    Puck::moveTo(inches);
}

std::vector<Point3<double>> Puck::estimateTrajectory(bool ignore_return) {
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
    std::vector<Point3<double>> trajectory;
    for (size_t i = 0; i < NUM_SAMPLES; i++) {
        double time = i * time_step;
        auto o = determineFutureOrientation(time);
        trajectory.push_back({o.first.x, o.first.y, time});
    }
    
    // Return trajectory
    return trajectory;
}

std::pair<Point2<double>, Point2<double>> Puck::determineFutureOrientation(double dt) {
    Point2<double> pos = _puck.position();
    Point2<double> vel = _puck.velocity();

    // Exit early if not moving
    if (vel.squaredMagnitude() <= Constants::FP_ERR)
        return {pos, vel};

    // Raw displacement
    Point2<double> new_pos = pos + dt * vel;

    // Effective puck bounds (accounting for the radius of the puck)
    Point2<double> min = Constants::Puck::RADIUS * Point2<double>::one();
    Point2<double> max = Constants::Table::SIZE - min;

    // Reflection variables
    Point2<double> range = max - min;
    Point2<double> period = 2.0 * range;
    Point2<double> mod(
        std::fmod(new_pos.x - min.x, period.x),
        std::fmod(new_pos.y - min.y, period.y)
    );

    // Reflect off x-axis
    if (mod.x < 0)
        mod.x += period.x;

    if (mod.x <= range.x)
        new_pos.x = min.x + mod.x;
    else {
        new_pos.x = max.x - (mod.x - range.x);
        vel.x *= -1;
    }

    // Reflect off y-axis
    if (mod.y < 0)
        mod.y += period.y;

    if (mod.y <= range.y)
        new_pos.y = min.y + mod.y;
    else {
        new_pos.y = max.y - (mod.y - range.y);
        vel.y *= -1;
    }

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

void Puck::orient(const Point2<double>& pos, const Point2<double>& vel) {
    _puck.orient(pos, vel);
}

Point2<double> Puck::position() {
    return _puck.position();
}

Point2<double> Puck::velocity() {
    return _puck.velocity();
}