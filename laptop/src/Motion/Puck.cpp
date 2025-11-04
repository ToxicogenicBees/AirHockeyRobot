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

Matrix<Point3<double>> Puck::estimateTrajectory(bool ignore_return) {
    Point2<double> pos = _puck.position();
    Point2<double> vel = _puck.velocity();

    // If the puck isn't moving, return early
    if (vel.magnitude() < 1e-8)
        return Matrix<Point3<double>>();

    // If the puck is moving away and it should be ignored, return early
    if (vel.y > 0 && ignore_return)
        return Matrix<Point3<double>>();

    // Determine how long the puck will move for
    double time_of_arrival = (vel.y > 0 ? 2 * Constants::Table::SIZE.y - pos.y : -pos.y) / vel.y;

    // Determine number of sample points
    const size_t NUM_SAMPLES = std::min(
        (size_t) (time_of_arrival / (1e-6 * Constants::SAMPLE_PERIOD)),
        Constants::MAX_SAMPLE_POINTS
    );
    
    double time_step = time_of_arrival / NUM_SAMPLES;
    
    // Calculate trajectory
    Matrix<Point3<double>> trajectory(NUM_SAMPLES);
    for (size_t i = 0; i < NUM_SAMPLES; i++) {
        double time = i * time_step;
        auto o = determineFutureOrientation(time);
        trajectory(i) = {o.first.x, o.first.y, time};
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
    Point2<double> raw_disp = pos + dt * vel;

    // Triangle-wave reflection
    auto reflect = [](double x, double T, double& v) {
        double period = 2.0 * T;
        double mod = std::fmod(x, period);
        if (mod < 0)
            mod += period;

        if (mod <= T)
            return mod;

        else {
            v = -v;
            return period - mod;
        }
    };

    Point2<double> future_pos(
        reflect(raw_disp.x, Constants::Table::SIZE.x, vel.x),
        reflect(raw_disp.y, Constants::Table::SIZE.y, vel.y)
    );

    return { future_pos, vel };
}

Point2<double> Puck::reflectedVelocity() {
    // v(T+) = (I - 2nn^T)v(T-)
    // v(T+) = v(T-) - 2(n ⋅ v(T-))n

    // Get the normal to the collision
    Point2<double> n = (position() - Mallet::position()).normal();
    Point2<double> v = velocity();

    // Return new velocity
    return Point2<double>(
        v.x - 2 * n.dot(v) * n.x,
        v.y - 2 * n.dot(v) * n.y
    );
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