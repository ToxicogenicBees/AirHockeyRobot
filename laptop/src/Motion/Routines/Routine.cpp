#include "Motion/Routines/Routine.h"
#include "Comms/SerialLink.hpp"
#include "Comms/Packet.hpp"
#include "Motion/Mallet.h"
#include "Constants.h"

double Routine::_timeToReach(const Point2<double>& position) {
    return (position - Mallet::position()).magnitude() / Constants::Mallet::SPEED;
}

bool Routine::_canReach(const Point2<double>& position) {
    // Ensure position is within reach
    if (position.x < Constants::Mallet::LIMIT_BL.x || position.y < Constants::Mallet::LIMIT_BL.y)
        return false;
    
    else if (position.x > Constants::Mallet::LIMIT_TR.x || position.y > Constants::Mallet::LIMIT_TR.y)
        return false;

    // Ensure the mallet has the time to reach this point
    if (_timeToReach(position) < 0)
        return false;
    
    return true;
}

Routine::Routine() : _target(Constants::Mallet::HOME) {}

Point2<double> Routine::getTarget() {
    return _target;
}

void Routine::transmitTarget() {
    // Buffer velocity profile
    Packet vel(Action::VelocityProfile);
    vel << _velocity_profile;
    SerialLink::buffer(vel);

    // Convert position to millimeters
    auto target_mm = 25.4 * (_target - Constants::Mallet::LIMIT_BL);
    
    // Buffer position
    Packet pos(Action::MalletPosition);
    pos << target_mm;
    SerialLink::buffer(pos);
}