#include "Routines/Routine.hpp"
#include "Comms/SerialLink.hpp"
#include "Comms/Packet.hpp"
#include "Types/Ray2.hpp"
#include "Constants.hpp"

#include <thread>

namespace {
    const double INV_SQRT_2 = 1.0 / sqrt(2.0);
    const Point2<double> A_AXIS = {INV_SQRT_2, INV_SQRT_2};
    const Point2<double> B_AXIS = {-INV_SQRT_2, INV_SQRT_2};
}

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

std::optional<StrikePlan> Routine::_planStrike(const Ray2<double>& orientation, double time) {
    if (time < 0)
        return std::nullopt;

    auto position = orientation.position;
    auto velocity = orientation.direction;

    auto speed = orientation.magnitude();
    auto accel_dist = speed / Constants::Mallet::MAX_SPEED_INCHES_PER_SECOND * (Constants::Mallet::INCHES_TO_ACCEL_TO_MAX_RPM - Constants::Mallet::MIN_ACCEL_INCHES) + Constants::Mallet::MIN_ACCEL_INCHES;

    Point2<double> setup_position = position - accel_dist * velocity.normal();
    Point2<double> strike_position = position + velocity.normal(); // 1 inch past

    // bounds check
    auto oob = [](const Point2<double>& p) {
        return p.x < Constants::Mallet::LIMIT_BL.x ||
               p.y < Constants::Mallet::LIMIT_BL.y ||
               p.x > Constants::Mallet::LIMIT_TR.x ||
               p.y > Constants::Mallet::LIMIT_TR.y;
    };

    if (oob(setup_position) || oob(strike_position))
        return std::nullopt;

    double min_speed = Constants::Mallet::MIN_RPM / Constants::Mallet::MAX_RPM * Constants::Mallet::MAX_SPEED_INCHES_PER_SECOND;
    double time_to_strike = accel_dist / (speed - min_speed) * log(1 + (speed - min_speed) / min_speed);

    if (time_to_strike > time)
        return std::nullopt;

    double time_to_setup = time - time_to_strike;

    // compute setup feasibility
    auto mallet_pos = _mallet->position();
    auto setup_displacement = setup_position - mallet_pos;
    auto setup_direction = setup_displacement.normal();

    double speed_to_setup = setup_displacement.magnitude() / time_to_setup;
    double rpm_to_setup = speed_to_setup / Constants::Mallet::MAX_SPEED_INCHES_PER_SECOND * Constants::Mallet::MAX_RPM;

    // scalar projection of setup position normal onto CoreXY movement axis
    // to get percent motor speed of the max speed
    double rpm_scale_a = fabs( setup_direction.scalarProjection(A_AXIS) );
    double rpm_scale_b = fabs( setup_direction.scalarProjection(B_AXIS) );
    rpm_to_setup *= rpm_scale_a > rpm_scale_b ? rpm_scale_a : rpm_scale_b;

    if (rpm_to_setup > Constants::Mallet::MIN_RPM)
        return std::nullopt;

    return StrikePlan{
        setup_position,
        time_to_setup,
        {strike_position, velocity},
        time_to_strike
    };
}

Routine::StrikeResult Routine::_strike(const Ray2<double>& orientation, double time) {
    if (time < 0) {
        return StrikeResult::STRIKE_IMPOSSIBLE;
    }

    // get setup point based on how long the gantry needs to
    // accelerate to the desired striking velocity

    // linearly interpolate from min to max distance to accelerate to max speed
    auto pos = orientation.position;
    auto vel = orientation.direction;
    auto speed = vel.magnitude();
    double accel_dist_inches = speed / Constants::Mallet::MAX_SPEED_INCHES_PER_SECOND * (Constants::Mallet::INCHES_TO_ACCEL_TO_MAX_RPM - Constants::Mallet::MIN_ACCEL_INCHES) + Constants::Mallet::MIN_ACCEL_INCHES;

    Point2<double> setup_point = pos - accel_dist_inches * vel.normal();

    // time it will take to get from setup_point to pos assuming constant acceleration
    // and starting from 0 velocity
    // double time_to_strike = sqrt(2*accel_dist_inches / Constants::Mallet::ACCEL) + 0.09;
    // double time_to_strike = 0.02;

    // assuming acceleration over a distance (Kaden's method)
    double min_speed = Constants::Mallet::MIN_RPM / Constants::Mallet::MAX_RPM * Constants::Mallet::MAX_SPEED_INCHES_PER_SECOND;
    double time_to_strike = accel_dist_inches / (speed - min_speed) * log(1 + (speed - min_speed) / min_speed);

    if (time_to_strike > time || time_to_strike < Constants::FP_ERR) {
        return StrikeResult::STRIKE_IMPOSSIBLE;
    }

    // finally set strike point to an inch past the input pos so can decel after hitting it
    // go through pos
    auto strike_point = orientation.unit().endPoint();

    // check for points out of bounds
    auto out_of_bounds = [](const Point2<double> p) -> bool {
        return p.x < Constants::Mallet::LIMIT_BL.x
            || p.y < Constants::Mallet::LIMIT_BL.y
            || p.x > Constants::Mallet::LIMIT_TR.x
            || p.y > Constants::Mallet::LIMIT_TR.y;
    };

    if (out_of_bounds(setup_point) || out_of_bounds(strike_point)) {
        return StrikeResult::STRIKE_IMPOSSIBLE;
    }

    // find time left to be able to go to setup point
    double time_to_setup = time - time_to_strike;

    if (time_to_setup < Constants::FP_ERR) {
        return StrikeResult::STRIKE_IMPOSSIBLE;
    }

    // find distance to go to setup point
    auto mallet_position = _mallet->position();
    auto setup_displacement = setup_point - mallet_position;
    auto setup_direction = setup_displacement.normal();

    // find required speed of motor with more steps to setup point
    double speed_to_setup = setup_displacement.magnitude() / time_to_setup;
    double rpm_to_setup = speed_to_setup / Constants::Mallet::MAX_SPEED_INCHES_PER_SECOND * Constants::Mallet::MAX_RPM;
    
    // scalar projection of setup position normal onto CoreXY movement axis
    // to get percent motor speed of the max speed
    double rpm_scale_a = fabs( setup_direction.scalarProjection(A_AXIS) );
    double rpm_scale_b = fabs( setup_direction.scalarProjection(B_AXIS) );
    rpm_to_setup *= rpm_scale_a > rpm_scale_b ? rpm_scale_a : rpm_scale_b;

    if (rpm_to_setup > Constants::Mallet::MIN_RPM) {
        return StrikeResult::STRIKE_IMPOSSIBLE;
    }

    // begin movement
    softTransmit({ 0, 0, (uint16_t) rpm_to_setup, (uint16_t) rpm_to_setup });
    softTransmit(setup_point);

    if (time < 0.15) {
        std::this_thread::sleep_for(std::chrono::microseconds((int64_t)(time_to_setup*1e6)));
        // after waiting, run the strike!
    } else if ((mallet_position - setup_point).magnitude() < 0.5) {
        // at setup position, run the strike!
    } else {
        return StrikeResult::STRIKE_IN_PROGRESS;
    }

    // strike
    double accel_percent = accel_dist_inches / (strike_point - setup_point).magnitude();
    double rpm_at_strike = speed / Constants::Mallet::MAX_SPEED_INCHES_PER_SECOND * Constants::Mallet::MAX_RPM;
    
    // scalar projection of position vector onto CoreXY movement axis
    // divide the scalar projection value over the total movement distance to
    // get percent motor speed of the max speed
    rpm_scale_a = fabs( (pos - setup_point).scalarProjection(A_AXIS) ) / accel_dist_inches;
    rpm_scale_b = fabs( (pos - setup_point).scalarProjection(B_AXIS) ) / accel_dist_inches;
    rpm_at_strike *= rpm_scale_a > rpm_scale_b ? rpm_scale_a : rpm_scale_b;

    softTransmit({ accel_percent, 0.05, (uint16_t) Constants::Mallet::MIN_RPM, (uint16_t) rpm_at_strike});
    softTransmit(strike_point);

    // wait to return control to main mallet control function until done
    std::this_thread::sleep_for(std::chrono::microseconds((int64_t)(time_to_strike*1e6)));

    return StrikeResult::STRIKE_COMPLETE;
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
    if ((position - _prev_target.first).magnitude() >= Constants::FP_ERR) {
        transmit(position);
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
