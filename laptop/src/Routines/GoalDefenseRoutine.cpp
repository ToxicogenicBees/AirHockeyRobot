#include <optional>
#include <vector>

#include "Routines/GoalDefenseRoutine.hpp"
#include "Motion/Table.hpp"
#include "Types/Point2.hpp"
#include "Constants.hpp"

namespace {
    
};

void GoalDefenseRoutine::updateTarget() {
    // Fetch puck timestamps
    auto timestamps = Table::puck().trajectory(true);

    Point2<double> defense_target;

    if (!timestamps.empty()) {
        defense_target = {timestamps.back().second.position.x, 2};
    } else {
        defense_target = Constants::Table::ROBOT_GOAL + Point2<double>::yAxis()*2;
    }

    if ( (Table::mallet().position() - _prev_target.first).magnitude() < 0.5 ) {
        if ( (Table::mallet().position() - defense_target).magnitude() > 12 ) {
            softTransmit({ 0.1, 0.1, 100, 600 }); 
        } else if ( (Table::mallet().position() - defense_target).magnitude() > 5 ) {
            softTransmit({ 0.1, 0.1, 100, 400 }); 
        } else {
            softTransmit({ 0.1, 0.1, 100, 200 }); 
        }

        softTransmit(defense_target);
    }
}
