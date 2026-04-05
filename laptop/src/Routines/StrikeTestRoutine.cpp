#include "Routines/StrikeTestRoutine.hpp"
#include "Comms/SerialLink.hpp"
#include "Comms/Packet.hpp"
#include "Motion/Table.hpp"
#include "Constants.hpp"

void StrikeTestRoutine::updateTarget() {
    // Fetch puck timestamps
    auto timestamps = Table::puck().trajectory();

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

        auto dist = (puck_position - mallet_position).magnitude();

        // Weight target
        auto weight = 
            -0.25 * margin;                                // Relative time between arrivals
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

    // Get desired striking velocity
    auto strike_velocity = 0.5 * Constants::Mallet::MAX_SPEED_INCHES_PER_SECOND * (Constants::Table::HUMAN_GOAL - best_target).normal();

    // Attempt strike
    auto result = _strike({best_target, strike_velocity}, best_time);

    // Strike is setting up, restart timer
    if (result == StrikeResult::STRIKE_IN_PROGRESS) {
        _timer.reset();
    }

    // Strike completed, return home
    else if (result == StrikeResult::STRIKE_COMPLETE || _timer.delta<std::chrono::milliseconds>() > 1e3) {
        _travelHome();
        return;
    }

    // Request distance sensor reading if near the target point
    if (distance < 0.20) {
        Packet request(Action::DistanceSensorRead);
        SerialLink::buffer(request);
    }
}
