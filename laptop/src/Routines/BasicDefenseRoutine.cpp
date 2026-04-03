#include "Routines/BasicDefenseRoutine.hpp"
#include "Motion/Table.hpp"
#include "Constants.hpp"

#include <algorithm>

namespace {
    constexpr double TARGET_ERROR = (0.05 * Constants::Mallet::RADIUS);
}

void BasicDefenseRoutine::updateTarget() {
    // Fetch puck timestamps
    auto timestamps = Table::puck().trajectory();

    // Set time for each trajectory to be the mallet's time of arrival
    double best_weight = -std::numeric_limits<double>::infinity();
    Point2<double> best_target = Constants::Mallet::HOME;
    Point2<double> mallet_position = _mallet->position();
    
    for (const auto& [time, orientation] : timestamps) {
        auto puck_position = orientation.position;

        // Ignore if unreachable
        if (!_canReach(puck_position))
            continue;

        // Parameters for choosing a target
        auto reach_time = _timeToReach(puck_position);

        auto margin = time - reach_time;
        if (margin < 0)
            continue;

        auto dist = (puck_position - mallet_position).magnitude();

        // Weight target
        auto weight = 
            -0.25 * margin;                                // Relative time between arrivals
            // + 0.5 / (1 + sqrt(t.z))                           // Time the puck arrives
            // + 0.75 / (1 + dist);   // The distance from the puck's current location (normalized for time)

        // Check if this weight is the best
        if (weight > best_weight) {
            best_weight = weight;
            best_target = {
                std::clamp(puck_position.x, Constants::Mallet::LIMIT_BL.x + 1, Constants::Mallet::LIMIT_TR.x - 1),
                std::clamp(puck_position.y, Constants::Mallet::LIMIT_BL.y + 1, Constants::Mallet::LIMIT_TR.y - 1)
            };
        }
    }
    
    // Only modify target if outside of acceptable tolerance
    if ((_prev_target.first - best_target).magnitude() > TARGET_ERROR) {
        // Set speed based on distance from the point
        auto displacement = (mallet_position - best_target).magnitude();
        if (displacement < 1)
            softTransmit({0, 0, 50, 50});
        else if (displacement < 2)
            softTransmit({0, 0, 150, 150});
        else if (displacement < 5)
            softTransmit({0.1, 0, 300, 500});
        else
            softTransmit({0.15, 0.15, 300, 650});

        // Set target
        softTransmit(best_target);
    }
}
