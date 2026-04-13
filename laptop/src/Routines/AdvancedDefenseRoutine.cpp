#include "Routines/AdvancedDefenseRoutine.hpp"
#include "Motion/Table.hpp"

void AdvancedDefenseRoutine::updateTarget() {
    // Ignore the puck if it is invalid
    if (!Table::puck().isValid()) {
        _travelHome();
        return;
    }

    // Get puck position
    auto puck_position = Table::puck().position();
    auto puck_velocity = Table::puck().velocity();

    // Dodge if puck is behind; otherwise play normal defense
    if ((_dodging && puck_velocity.y < 0) || puck_position.y < _mallet->position().y) {
        _dodge.updateTarget();
        _dodging = true;
    }
    else {
        _defense.updateTarget();
        _dodging = false;
    }
}