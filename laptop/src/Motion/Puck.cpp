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
    Point2<double> inches = Constants::PUCK_HOME;

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
        // Step forward
        Point2<double> new_est_pos = est_pos + est_vel * timestep;

        // Adjust for overshoot when bouncing
        if (new_est_pos.x < Constants::PUCK_RADIUS) {
            est_pos.x = 2 * Constants::PUCK_RADIUS - new_est_pos.x;
            est_pos.y = new_est_pos.y;
            est_vel.x = -est_vel.x;
        }

        else if (new_est_pos.x > Constants::TABLE_SIZE.x - Constants::PUCK_RADIUS) {
            est_pos.x = 2 * (Constants::TABLE_SIZE.x - Constants::PUCK_RADIUS) - new_est_pos.x;
            est_pos.y = new_est_pos.y;
            est_vel.x = -est_vel.x;
        }

        else {
            est_pos = new_est_pos;
        }

        // Add sample point to matrix
        samples(i) = Point3<double>(est_pos.x, est_pos.y, (i + 1) * timestep);
    }

    // Return sample points
    return samples;
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