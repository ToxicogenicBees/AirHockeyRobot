#include <optional>
#include <vector>

#include "Routines/BasicOffenseRoutine.hpp"
#include "Motion/Table.hpp"
#include "Types/Point2.hpp"
#include "Constants.hpp"

namespace {
    // Maximum number of bounces
    const int MAX_BOUNCES = 3;
    
    // Allowed collision normals
    const double INV_SQRT_2 = 1.0 / sqrt(2.0);
    std::vector<Point2<double>> NORMALS = {
        {-INV_SQRT_2, INV_SQRT_2},      // Up + left
        {INV_SQRT_2, INV_SQRT_2},       // Up + right
        Point2<double>::yAxis(),        // Up
    };

    // Scoring trajectory stats
    struct ScoringTrajectory {
        Point2<double> strike_position;
        Point2<double> strike_velocity;
        double strike_time;
    };
};

void BasicOffenseRoutine::updateTarget() {
    // Ignore if the puck is moving too fast (~1.5 m/s)
    if (Table::puck().velocity().magnitude() >= 60) {
        _travelHome();
        return;
    }

    // Calculate all trajectories
    auto trajectory = Table::puck().trajectory();
    for (auto [time, orientation] : trajectory) {
        // Get puck position and velocity
        auto puck_position = orientation.position;
        auto puck_velocity = orientation.direction;

        // Ensure the mallet can reach this point in the desired time
        if (!_canReach(puck_position))
            continue;

        // Check each allowed normal
        for (const auto& normal : NORMALS) {
            // Calculate unit tangent
            Point2<double> tangent = {-normal.y, normal.x};

            // Get parallel + tangent puck velocities
            auto parallel_puck_velocity = puck_velocity.projection(normal);
            auto tangent_puck_velocity = puck_velocity.projection(tangent);

            // Check each allowed bounces
            for (int bounce = -MAX_BOUNCES; bounce <= MAX_BOUNCES; ++bounce) {
                // Projected goal based on bounce count
                auto goal = Constants::Table::HUMAN_GOAL
                    + bounce * Constants::Table::SIZE.x * Point2<double>::xAxis();

                // Get displacement vector from the goal
                auto displacement = goal - puck_position;

                // Ensure the puck's tangential motion is parallel to the tangent displacement
                auto sign = [](double val) {
                    if (val > 0)
                        return 1;
                    else if (val < 0)
                        return -1;
                    return 0;
                };

                // Ignore bounce cases if the puck will not bounce off the desired wall
                auto tangent_displacement = displacement.projection(tangent);
                if (sign(tangent_puck_velocity.dot(tangent_displacement)) != 1)
                    continue;
                auto tangent_speed = tangent_puck_velocity.magnitude();
                if (bounce == 0 && std::abs(tangent_speed) < Constants::FP_ERR)
                    continue;

                // Fetch desired post-collision velocity
                auto travel_time = tangent_displacement.magnitude() / tangent_speed;
                if (travel_time <= Constants::FP_ERR)
                    continue;
                auto post_collision_velocity = displacement / travel_time;

                // Determine required mallet speed + position for this velocity
                auto mallet_velocity = 0.5 * (puck_velocity + post_collision_velocity).projection(normal);

                // Ignore motions that move the puck anti-parallel to the normal
                if (mallet_velocity.dot(normal) < 0)
                    continue;

                // Attempt this strike
                auto success = strike({puck_position, mallet_velocity}, time);

                // No need to continue checking possible strikes
                if (success) {
                    return;
                }
            }
        }
    }

    // Strike wasn't possible
    _travelHome();
}
