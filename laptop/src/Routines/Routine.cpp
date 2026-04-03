#include "Routines/Routine.hpp"
#include "Comms/SerialLink.hpp"
#include "Comms/Packet.hpp"
#include "Types/Ray2.hpp"
#include "Constants.hpp"

#include <thread>

namespace {
    const Point2<double> A_AXIS = {1/sqrt(2), 1/sqrt(2)};
    const Point2<double> B_AXIS = {-1/sqrt(2), 1/sqrt(2)};
}

double Routine::_timeToReach(const Point2<double>& position) {
    return (position - _mallet.position()).magnitude() / Constants::Mallet::SPEED;
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
    double accel_dist_inches = speed / Constants::Mallet::MAX_SPEED_INCHES_PER_SECOND * (Constants::Mallet::INCHES_TO_ACCEL_TO_MAX_RPM - Constants::Mallet::MIN_ACCEL_INCES) + Constants::Mallet::MIN_ACCEL_INCES;

    Point2<double> setup_point = pos - accel_dist_inches * vel.normal();

    // time it will take to get from setup_point to pos assuming constant acceleration
    // and starting from 0 velocity
    // double time_to_strike = sqrt(2*accel_dist_inches / Constants::Mallet::ACCEL) + 0.09;
    // double time_to_strike = 0.02;

    // assuming acceleration over a distance (Kaden's method)
    double min_speed = Constants::Mallet::MIN_RPM / Constants::Mallet::MAX_RPM * Constants::Mallet::MAX_SPEED_INCHES_PER_SECOND;
    double time_to_strike = accel_dist_inches / (speed - min_speed) * log(1 + (speed - min_speed) / min_speed);

    if (time_to_strike > time) {
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

    if (time_to_setup < 0) {
        return StrikeResult::STRIKE_IMPOSSIBLE;
    }

    // find distance to go to setup point
    auto mallet_position = _mallet.position();
    auto setup_displacement = setup_point - mallet_position;
    auto setup_direction = setup_displacement.normal();

    // find required speed of motor with more steps to setup point
    double speed_to_setup = setup_displacement.magnitude() / time_to_setup;
    double rpm_to_setup = speed_to_setup / Constants::Mallet::MAX_SPEED_INCHES_PER_SECOND * Constants::Mallet::MAX_RPM;
    
    // scalar projection of setup position normal onto CoreXY movement axis
    // to get percent motor speed of the max speed
    double rpm_scale_a = abs( setup_direction.scalarProjection(A_AXIS) );
    double rpm_scale_b = abs( setup_direction.scalarProjection(B_AXIS) );
    rpm_to_setup *= rpm_scale_a > rpm_scale_b ? rpm_scale_a : rpm_scale_b;

    if (rpm_to_setup > Constants::Mallet::MIN_RPM)
        return StrikeResult::STRIKE_IMPOSSIBLE;

    // begin movement
    _velocity_profile = { 0, 0, (uint16_t) rpm_to_setup, (uint16_t) rpm_to_setup };
    _target = setup_point;
    transmitTarget();

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
    rpm_scale_a = abs( (pos - setup_point).scalarProjection(A_AXIS) ) / accel_dist_inches;
    rpm_scale_b = abs( (pos - setup_point).scalarProjection(B_AXIS) ) / accel_dist_inches;
    rpm_at_strike *= rpm_scale_a > rpm_scale_b ? rpm_scale_a : rpm_scale_b;

    _velocity_profile = { accel_percent, 0.05, (uint16_t) Constants::Mallet::MIN_RPM, (uint16_t) rpm_at_strike};
    _target = strike_point;
    transmitTarget();

    // wait to return control to main mallet control function until done
    std::this_thread::sleep_for(std::chrono::microseconds((int64_t)(time_to_strike*1e6)));

    return StrikeResult::STRIKE_COMPLETE;
}

void Routine::_targetHome() {
    _velocity_profile = { 0, 0, 250, 250 };
    _target = Constants::Mallet::HOME;
}

Routine::Routine(MovingObject& mallet) : _mallet(mallet), _target(Constants::Mallet::HOME) {}

Point2<double> Routine::target() const {
    return _target;
}

void Routine::transmitTarget() const {
    // Buffer velocity profile
    Packet vel(Action::VelocityProfile);
    vel << _velocity_profile;
    SerialLink::buffer(vel);

    // Convert position to millimeters
    auto target_mm = 25.4 * (_target - Constants::Mallet::SENSOR_OFFSET);
    
    // Buffer position
    Packet pos(Action::MalletPosition);
    pos << target_mm;
    SerialLink::buffer(pos);
}
