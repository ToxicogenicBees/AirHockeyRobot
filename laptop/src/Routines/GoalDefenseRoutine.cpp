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
    // if puck moving very slowly, go to meet it to keep gameplay going
    if (Table::puck().velocity().magnitude() < 5) {
        softTransmit({ 0.1, 0.1, 50, 100 });
        softTransmit(Table::puck().position());

        Timer timeout;
        while ((_mallet->position() - _prev_target.first).magnitude() > 0.1 && timeout.delta<std::chrono::milliseconds>() < 250) {
            std::this_thread::sleep_for(std::chrono::microseconds((int64_t)(1)));
        }
        if ((_mallet->position() - _prev_target.first).magnitude() < 0.5) {
            SerialLink::buffer({Action::DistanceSensorRead});
        }

        return;
    }

    // Otherwise, calculate the puck trajectory and attempt to block it
    auto timestamps = Table::puck().trajectory();
    if (!timestamps.empty() && timestamps.back().second.direction.magnitude() > 5) {
        Point2<double> defense_target = {timestamps.back().second.position.x, 2};

        if ( (defense_target - _prev_target.first).magnitude() > Constants::Mallet::RADIUS/2 ) {
            if ( (_mallet->position() - defense_target).magnitude() > 12 ) {
                softTransmit({ 0.1, 0.1, 100, 600 }); 
            } else if ( (_mallet->position() - defense_target).magnitude() > 8 ) {
                softTransmit({ 0.1, 0.1, 100, 400 }); 
            } else {
                softTransmit({ 0.1, 0.1, 100, 150 }); 
            }

            softTransmit(defense_target);

            Timer timeout;
            while ((_mallet->position() - _prev_target.first).magnitude() > 0.1 && timeout.delta<std::chrono::milliseconds>() < 250) {
                std::this_thread::sleep_for(std::chrono::microseconds((int64_t)(1)));
            }
            if ((_mallet->position() - _prev_target.first).magnitude() < 0.1) {
                SerialLink::buffer({Action::DistanceSensorRead});
            }
        }
    }

    else {
        Point2<double> defense_target = Constants::Table::ROBOT_GOAL + 2.0 * Point2<double>::yAxis();

        if ( (defense_target - _prev_target.first).magnitude() > Constants::Mallet::RADIUS/2 ) {
            auto displacement = (_mallet->position() - defense_target).magnitude();
            if ( displacement > 12 ) {
                softTransmit({ 0.1, 0.1, 100, 600 }); 
            } else if ( displacement > 8 ) {
                softTransmit({ 0.1, 0.1, 100, 400 }); 
            } else {
                softTransmit({ 0.1, 0.1, 100, 150 }); 
            }

            softTransmit(defense_target);

            Timer timeout;
            while ((_mallet->position() - _prev_target.first).magnitude() > 0.1 && timeout.delta<std::chrono::milliseconds>() < 250) {
                std::this_thread::sleep_for(std::chrono::microseconds((int64_t)(1)));
            }
            if ((_mallet->position() - _prev_target.first).magnitude() < 0.1) {
                SerialLink::buffer({Action::DistanceSensorRead});
            }
        }
    }
}
