#include "Motion/Routines/BasicDefenseRoutine.h"
#include "Motion/Mallet.h"
#include "Motion/Puck.h"
#include "Constants.h"

#include <algorithm>

namespace {
    constexpr double TARGET_ERROR = (0.05 * Constants::Mallet::RADIUS);
}

BasicDefenseRoutine::BasicDefenseRoutine() : Routine() {}

std::unique_ptr<Routine> BasicDefenseRoutine::clone() const {
    return std::make_unique<BasicDefenseRoutine>(*this);
}

void BasicDefenseRoutine::updateTarget() {
    // Create copy of timestamps
    auto timestamps = Puck::estimateTrajectory();

    // Set time for each trajectory to be the mallet's time of arrival
    double best_weight = -std::numeric_limits<double>::infinity();
    Point2<double> best_target = Constants::Mallet::HOME;
    Point2<double> mallet_position = Mallet::position();
    
    for (const auto& [orientation, time] : timestamps) {
        auto puck_position = orientation.position();

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
    if ((_target - best_target).magnitude() > TARGET_ERROR) {
        _target = best_target;

        auto dist_mag = (mallet_position - _target).magnitude();

        if (dist_mag < 1)
            _velocity_profile = VelocityProfile(0, 0, 50, 50);
        else if (dist_mag < 2)
            _velocity_profile = VelocityProfile(0, 0, 150, 150);
        else if (dist_mag < 5)
            _velocity_profile = VelocityProfile(0.1, 0, 350, 500);
        else if (dist_mag < 15)
            _velocity_profile = VelocityProfile(0.15, 0, 500, 650);
    }
}