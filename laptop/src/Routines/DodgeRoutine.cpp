#include "Routines/DodgeRoutine.hpp"
#include "Motion/Table.hpp"
#include "Constants.hpp"

#include <algorithm>

void DodgeRoutine::updateTarget() {
    // Fetch puck position
    auto puck_position = Table::puck().position();

    // Fetch puck trajectory
    auto trajectory = Table::puck().trajectory(true);

    // Escape early if there is no trajectory
    if (trajectory.empty()) {
        return;
    }

    // Get puck position when it hits the back wall
    auto [_, puck_end_location] = trajectory[trajectory.size() - 1];
    auto puck_end_position = puck_end_location.position;

    // Choose a dodge location and go there
    softTransmit(puck_end_position.x > Constants::Table::ROBOT_GOAL.x
        ? Constants::Mallet::DODGE_LEFT
        : Constants::Mallet::DODGE_RIGHT
    );
    softTransmit({0.15, 0.15, 300, 650});
}
