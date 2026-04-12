#include "Routines/StrikeTestRoutine.hpp"
#include "Comms/SerialLink.hpp"
#include "Comms/Packet.hpp"
#include "Motion/Table.hpp"
#include "Constants.hpp"

void StrikeTestRoutine::updateTarget() {
    // Fetch puck timestamps
    auto timestamps = Table::puck().trajectory(true);

    // Set time for each trajectory to be the mallet's time of arrival
    double best_weight = -std::numeric_limits<double>::infinity();
    double best_time = -std::numeric_limits<double>::infinity();
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

        double desired_margin = 1.5;

        // auto dist = (puck_position - Constants::Mallet::HOME + 5.0*Point2<double>::yAxis()).magnitude();
        auto dist = (puck_position - Point2<double>{Constants::Table::ROBOT_GOAL.x, Constants::Mallet::LIMIT_TR.y - 6.0}).magnitude();
        double desired_dist = 5; // desired within radius 5

        // Weight target
        auto weight = 
            -0.95 * dist/desired_dist;
            -0.05 * margin/desired_margin;                                // Relative time between arrivals
            // + 0.5 / (1 + sqrt(t.z))                           // Time the puck arrives
            // + 0.75 / (1 + dist);   // The distance from the puck's current location (normalized for time)

        // Check if this weight is the best
        if (weight > best_weight) {
            best_weight = weight;
            best_time = time;
            best_target = {
                std::clamp(puck_position.x, Constants::Mallet::LIMIT_BL.x + 1, Constants::Mallet::LIMIT_TR.x - 1),
                std::clamp(puck_position.y, Constants::Mallet::LIMIT_BL.y + 1, Constants::Mallet::LIMIT_TR.y - 1)
            };
        }
    }
    
    // Save info of this new target
    auto distance = (_mallet->position() - best_target).magnitude();
    auto puck_velocity = Table::puck().velocity();

    if (best_target.y > Constants::Mallet::HOME.y - 5) {
        // Get desired striking velocity
        auto strike_velocity = 0.5 * Constants::Mallet::MAX_SPEED_INCHES_PER_SECOND * (Constants::Table::HUMAN_GOAL - best_target).normal();

        // Attempt the strike, or simply move home or to defend if it fails
        auto success = strike({best_target, strike_velocity}, {best_target, puck_velocity}, best_time);
        if (!success) {
            // _travelHome();
            // softTransmit({ 0.1, 0.1, 100, 500 });
            // softTransmit(Constants::Table::ROBOT_GOAL + Point2<double>::yAxis()*2);

            _goal_defense.updateTarget();

            if ((Table::mallet().position() - _prev_target.first).magnitude() < 0.1) {
                SerialLink::buffer({Action::DistanceSensorRead});
            }
        }
    } else {
        _goal_defense.updateTarget();

        if ((Table::mallet().position() - _prev_target.first).magnitude() < 0.1) {
            SerialLink::buffer({Action::DistanceSensorRead});
        }
    }
}
