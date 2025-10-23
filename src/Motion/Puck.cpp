#include "../../include/Motion/Puck.h"

void Puck::readPosition(const Point2<double>& new_pos, double time_step) {
    _prev_pos = _cur_pos;
    _cur_pos = new_pos;

    _velocity = (_cur_pos - _prev_pos) / time_step;
}

Matrix<Point3<double>> Puck::estimateTrajectory(size_t num_points) {
    // Ignore trajectories where the puck is moving away (return empty matrix)
    if (_velocity.y > 0)
        return Matrix<Point3<double>>();

    Matrix<Point3<double>> samples(num_points);

    // Determine how long each step needs to be
    double time_to_travel = -_cur_pos.y / _velocity.y;
    double time_step = time_to_travel / num_points;

    // Calculate trajectory path
    Point2<double> est_vel = _velocity;
    Point2<double> est_pos = _cur_pos;

    // Determine sample points
    for (size_t i = 0; i < num_points; i++) {
        // Step forward
        Point2<double> new_est_pos = est_pos + est_vel * time_step;

        // Adjust for overshoot when bouncing
        if (new_est_pos.x < _RADIUS) {
            est_pos.x = 2*_RADIUS - new_est_pos.x;
            est_pos.y = new_est_pos.y;
            est_vel.x = -est_vel.x;
        }

        else if (new_est_pos.x > Constants::TABLE_SIZE.x - _RADIUS) {
            est_pos.x = 2 * (Constants::TABLE_SIZE.x - _RADIUS) - new_est_pos.x;
            est_pos.y = new_est_pos.y;
            est_vel.x = -est_vel.x;
        }

        else {
            est_pos = new_est_pos;
        }

        // Add sample point to matrix
        samples(i) = Point3<double>(est_pos.x, est_pos.y, (i + 1) * time_step);
    }

    // Return sample points
    return samples;
}