#include <opencv2/opencv.hpp>

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

Matrix<Point3<double>> Puck::estimateTrajectory() {
    Point2<double> est_vel = _puck.velocity();
    Point2<double> est_pos = _puck.position();

    // Ignore trajectories where the puck is moving away (return empty matrix)
    if (est_vel.y > 0)
        return Matrix<Point3<double>>();

    // Ignore if puck isn't moving
    if (std::abs(est_vel.y) < 1e-10)
        return Matrix<Point3<double>>();

    Matrix<Point3<double>> samples(Constants::NUM_SAMPLE_POINTS);

    // Determine how long each step needs to be
    double time_to_travel = -est_pos.y / est_vel.y;
    double timestep = time_to_travel / Constants::NUM_SAMPLE_POINTS;

    // Calculate trajectory path
    // Determine sample points
    for (size_t i = 0; i < Constants::NUM_SAMPLE_POINTS; i++) {
        // Determine trajectory
        auto est_orientation = determineFutureOrientation(i * timestep);

        // Add sample point to matrix
        samples(i) = Point3<double>(est_orientation.first.x, est_orientation.first.y, (i + 1) * timestep);
    }

    // Return sample points
    return samples;
}

std::pair<Point2<double>, Point2<double>> Puck::determineFutureOrientation(double dt) {
    Point2<double> est_pos = _puck.position();
    Point2<double> est_vel = _puck.velocity();

    // Exit early if not moving
    if (est_vel.squaredMagnitude() <= 1e-8)
        return {est_pos, est_vel};

    // Step forward
    Point2<double> new_est_pos = est_pos + est_vel * dt;

    // x-axis bounce
    if (new_est_pos.x < Constants::Puck::RADIUS) {
        new_est_pos.x = Constants::Puck::RADIUS + (Constants::Puck::RADIUS - new_est_pos.x);
        est_vel.x = -est_vel.x;
    }
    else if (new_est_pos.x > Constants::Table::SIZE.x - Constants::Puck::RADIUS) {
        new_est_pos.x = (Constants::Table::SIZE.x - Constants::Puck::RADIUS) - (new_est_pos.x - (Constants::Table::SIZE.x - Constants::Puck::RADIUS));
        est_vel.x = -est_vel.x;
    }

    // y-axis bounce
    if (new_est_pos.y < Constants::Puck::RADIUS) {
        new_est_pos.y = Constants::Puck::RADIUS + (Constants::Puck::RADIUS - new_est_pos.y);
        est_vel.y = -est_vel.y;
    }
    else if (new_est_pos.y > Constants::Table::SIZE.y - Constants::Puck::RADIUS) {
        new_est_pos.y = (Constants::Table::SIZE.y - Constants::Puck::RADIUS) - (new_est_pos.y - (Constants::Table::SIZE.y - Constants::Puck::RADIUS));
        est_vel.y = -est_vel.y;
    }

    // Set final position
    est_pos = new_est_pos;

    return {est_pos, est_vel};
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