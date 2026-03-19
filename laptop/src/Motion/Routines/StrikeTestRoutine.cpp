#include "Motion/Routines/StrikeTestRoutine.h"
#include "Comms/SerialLink.hpp"
#include "Comms/Packet.hpp"
#include "Motion/Mallet.h"
#include "Motion/Puck.h"
#include "Constants.h"

StrikeTestRoutine::StrikeTestRoutine() : BasicDefenseRoutine() {}

std::unique_ptr<Routine> StrikeTestRoutine::clone() const {
    return std::make_unique<StrikeTestRoutine>(*this);
}

void StrikeTestRoutine::updateTarget() {
    // Set up default defense location
    BasicDefenseRoutine::updateTarget();

    // Save info of this new target
    auto distance = (Mallet::position() - _target).magnitude();

    auto puck_velocity = Puck::velocity();
    auto time_to_reach = _target.y / puck_velocity.y;

    // Get desired striking velocity
    auto strike_velocity = 0.5 * Constants::Mallet::MAX_SPEED_INCHES_PER_SECOND * (Constants::Table::HUMAN_GOAL - _target).normal();

    // Attempt strike
    auto result = _strike({_target, strike_velocity}, time_to_reach);

    // Strike is setting up, restart timer
    if (result == StrikeResult::STRIKE_IN_PROGRESS) {
        _timer.reset();
    }

    // Strike completed, return home
    else if (result == StrikeResult::STRIKE_COMPLETE || _timer.delta<std::chrono::milliseconds>() > 1e3) {
        _targetHome();
        return;
    }

    // Request distance sensor reading if near the target point
    if (distance < 0.20) {
        Packet request(Action::DistanceSensorRead);
        SerialLink::buffer(request);
    }
}