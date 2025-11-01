#include "Motion/Puck.h"

Matrix<Point3<double>> Puck::estimateTrajectory(size_t num_points) {
    // Ignore trajectories where the puck is moving away (return empty matrix)
    if (_vel.y > 0)
        return Matrix<Point3<double>>();

    Matrix<Point3<double>> samples(num_points);

    // Determine how long each step needs to be
    double time_to_travel = -_pos.y / _vel.y;
    double timestep = time_to_travel / num_points;

    // Calculate trajectory path
    Point2<double> est_vel = _vel;
    Point2<double> est_pos = _pos;

    // Determine sample points
    for (size_t i = 0; i < num_points; i++) {
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