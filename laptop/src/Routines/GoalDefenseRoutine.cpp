#include <optional>
#include <vector>
#include <thread>

#include "Routines/GoalDefenseRoutine.hpp"
#include "Motion/Table.hpp"
#include "Types/Point2.hpp"
#include "Constants.hpp"
#include "Comms/SerialLink.hpp"

namespace {
    
};

void GoalDefenseRoutine::updateTarget() {
    // Fetch puck timestamps
    auto timestamps = Table::puck().trajectory();

    Point2<double> defense_target;

    if (!timestamps.empty()) {
        defense_target = {timestamps.back().second.position.x, 2};
    } else {
        defense_target = Constants::Table::ROBOT_GOAL + Point2<double>::yAxis()*2;
    }

    if ( (defense_target - _prev_target.first).magnitude() > Constants::Mallet::RADIUS/2 ) {
        if ( (Table::mallet().position() - defense_target).magnitude() > 12 ) {
            softTransmit({ 0.1, 0.1, 100, 600 }); 
        } else if ( (Table::mallet().position() - defense_target).magnitude() > 8 ) {
            softTransmit({ 0.1, 0.1, 100, 400 }); 
        } else {
            softTransmit({ 0.1, 0.1, 100, 150 }); 
        }

        softTransmit(defense_target);

        Timer timeout;

        while ((Table::mallet().position() - _prev_target.first).magnitude() > 0.1 && timeout.delta<std::chrono::milliseconds>() < 250) {
            std::this_thread::sleep_for(std::chrono::microseconds((int64_t)(1)));
        }

        if ((Table::mallet().position() - _prev_target.first).magnitude() < 0.1) {
            SerialLink::buffer({Action::DistanceSensorRead});
        }
    }

    // if puck moving very slowly, go to meet it to keep gameplay going
    if (Table::puck().velocity().magnitude() < 5) {
        // std::clog << "Hello there!\n\n";
        
        softTransmit({ 0.1, 0.1, 50, 100 }); 
        softTransmit(Table::puck().position());

        Timer timeout;

        while ((Table::mallet().position() - _prev_target.first).magnitude() > 0.1 && timeout.delta<std::chrono::milliseconds>() < 250) {
            std::this_thread::sleep_for(std::chrono::microseconds((int64_t)(1)));
        }

        if ((Table::mallet().position() - _prev_target.first).magnitude() < 0.5) {
            SerialLink::buffer({Action::DistanceSensorRead});
        }
    }
}
