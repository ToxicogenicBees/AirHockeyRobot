#include "Routines/Routine.hpp"
#include "Comms/SerialLink.hpp"
#include "Comms/Packet.hpp"
#include "Types/Ray2.hpp"
#include "Constants.hpp"

double Routine::_timeToReach(const Point2<double>& position) {
    return (position - _mallet->position()).magnitude() / Constants::Mallet::SPEED;
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

void Routine::_travelHome() {
    softTransmit(Constants::Mallet::HOME);
    softTransmit({ 0, 0, 250, 250 });
}

void Routine::setMallet(MovingObject* mallet) {
    _mallet = mallet;
}

Routine::Target Routine::target() {
    return _prev_target;
}

void Routine::softTransmit(const Target& target) {
    softTransmit(target.first);
    softTransmit(target.second);
}

void Routine::softTransmit(const Point2<double>& position) {
    Point2<double> p = position;

    if ((p - _prev_target.first).magnitude() >= Constants::FP_ERR) {
        // clamp position to robot bounds of movement
        if (p.x < Constants::Mallet::LIMIT_BL.x) {
            p.x = Constants::Mallet::LIMIT_BL.x;
        } else if (p.x > Constants::Mallet::LIMIT_TR.x) {
            p.x = Constants::Mallet::LIMIT_TR.x;
        }

        if (p.y < Constants::Mallet::LIMIT_BL.y) {
            p.y = Constants::Mallet::LIMIT_BL.y;
        } else if (p.y > Constants::Mallet::LIMIT_TR.y) {
            p.y = Constants::Mallet::LIMIT_TR.y;
        }

        transmit(p);
    }
}

void Routine::softTransmit(const VelocityProfile& velocity) {
    if (velocity.getAccelPercent() - _prev_target.second.getAccelPercent() >= Constants::FP_ERR
        || velocity.getDecelPercent() - _prev_target.second.getDecelPercent() >= Constants::FP_ERR
        || velocity.getMaxRPM() != _prev_target.second.getMaxRPM()
        || velocity.getMinRPM() != _prev_target.second.getMinRPM()) {

        transmit(velocity);
    }
}

void Routine::transmit(const Target& target) {
    transmit(target.first);
    transmit(target.second);
}

void Routine::transmit(const Point2<double>& position) {
    // Convert position to millimeters
    auto target_mm = 25.4 * (position - Constants::Mallet::SENSOR_OFFSET);
    
    // Buffer position
    Packet pos(Action::MalletPosition);
    pos << target_mm;
    SerialLink::buffer(pos);
    
    // Update previous position
    _prev_target.first = position;
}

void Routine::transmit(const VelocityProfile& velocity) {
    // Buffer velocity profile
    Packet vel(Action::VelocityProfile);
    vel << velocity;
    SerialLink::buffer(vel);

    // Update previous velocity
    _prev_target.second = velocity;
}
