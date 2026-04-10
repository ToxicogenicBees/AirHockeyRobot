#include "Motion/Gantry.hpp"
#include "Pinout.hpp"
#include "Constants.hpp"
#include "Comms/SerialLink.hpp"

#include <functional>

namespace {
    constexpr double STEP_CONVERSION_CONST = Motor::MICROSTEPS_PER_REV  // for converting delta X or Y to steps
        / (2*PI * Constants::Mallet::DRIVE_PULLEY_RADIUS_MM);
    
    bool step_motion_parity = false;

    /**
     * @brief Map an integer from one range to another
     * 
     * @param x         The integer to map
     * @param in_min    The lower input range
     * @param in_max    The upper input range
     * @param out_min   The lower output range
     * @param out_max   The upper output range
     * 
     * @return The mapped integer
     */
    int mapInt(int x, int in_min, int in_max, int out_min, int out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
};

Motor Gantry::_left(motor_l_step, motor_l_dir, motor_l_scs, motor_l_fault, motor_sleep, motor_enable);
Motor Gantry::_right(motor_r_step, motor_r_dir, motor_r_scs, motor_r_fault, motor_sleep, motor_enable);

Point2<double> Gantry::_position_offset = Constants::Mallet::HOME;

Point2<double> Gantry::_current_target;
Point2<int> Gantry::_total_steps_to_target;
int Gantry::_total_steps_larger = 0;
int Gantry::_step_counter = 0;
int Gantry::_accel_steps = 0;
int Gantry::_decel_steps = 0;
int Gantry::_current_rpm = 0;
Point2<int> Gantry::_steps = {0, 0};
Point2<int> Gantry::_d = {0, 0};
int Gantry::_err = 0;
uint16_t Gantry::_current_period_us = 0;

VelocityProfile Gantry::_profile;

GantryTimer Gantry::_timer(TIM3);

uint16_t Gantry::_step_period_from_rpm_over_two[1201];   // max rpm is 1200

void Gantry::init() {
    // Initialize motors
    _right.init();
    delay(100);
    _left.init();

    // Initialize hardware timer for motor step signal
    _timer.init(_stepMotion);

    // Populate rpm step period lookup table
    _step_period_from_rpm_over_two[0] = std::numeric_limits<uint16_t>::max();
    for (int i = 1; i <= 1200; ++i) {
        // steps/sec = (RPM / 60) * steps_per_rev
        double microsteps_per_second = (i / 60.0) * Motor::MICROSTEPS_PER_REV;
        _step_period_from_rpm_over_two[i] = (uint16_t) (0.5 * 1e6 / microsteps_per_second);
    }
}

void Gantry::setPosition(const Point2<double>& pos) {
    _position_offset = pos;
    _steps = Point2<int>::zero();
}

Point2<double> Gantry::getPosition() {
    return _position_offset + Point2<double>{
        0.5 * (_steps.x + _steps.y) / STEP_CONVERSION_CONST,
        0.5 * (_steps.x - _steps.y) / STEP_CONVERSION_CONST,
    };
}

void Gantry::setVelocityProfile(const VelocityProfile& profile) {
    _profile = profile;
}

Point2<int> Gantry::_calculateSteps(const Point2<double>& target) {
    auto step_displacement = STEP_CONVERSION_CONST * (target - getPosition());

    return {
        (int) (step_displacement.x + step_displacement.y),
        (int) (step_displacement.x - step_displacement.y)
    };
}

void Gantry::initMotion(const Point2<double>& target) {
    // Disable timer
    _timer.stop();
    __disable_irq();

    // Update target
    _current_target = target;

    // Update step counters
    _total_steps_to_target = _calculateSteps(_current_target);
    _total_steps_larger = std::abs(_total_steps_to_target.x) > std::abs(_total_steps_to_target.y)
        ? std::abs(_total_steps_to_target.x)
        : std::abs(_total_steps_to_target.y);
    _step_counter = 0;

    if (_total_steps_larger == 0)
        return;

    // Set motor directions
    _left.setDir(_total_steps_to_target.x > 0);
    _right.setDir(_total_steps_to_target.y > 0);

    // Calculate acceleration steps
    _accel_steps = _total_steps_larger * _profile.getAccelPercent();
    _decel_steps = _total_steps_larger * _profile.getDecelPercent();
    
    // Ensure cruise phase exists
    if (_accel_steps + _decel_steps > _total_steps_larger) {
        _accel_steps = _decel_steps = _total_steps_larger / 2;
    }

    // Calculate errors
    _d = {std::abs(_total_steps_to_target.x), -std::abs(_total_steps_to_target.y)};
    _err = _d.x + _d.y;

    // Start timer
    __enable_irq();
    _timer.start();
}

void Gantry::_stepMotion() {
    // Stop motors every other call
    if (step_motion_parity = !step_motion_parity) {
        _left.stepLow();
        _right.stepLow();

        return;
    }

    // Otherwise, step forward
    ++_step_counter;

    // Acceleration phase
    if (_step_counter < _accel_steps) {
        _current_rpm = mapInt(_step_counter, 0, _accel_steps, _profile.getMinRPM(), _profile.getMaxRPM());
    }

    // Deceleration Phase
    else if (_step_counter >= (_total_steps_larger - _decel_steps)) {
        _current_rpm = mapInt(_step_counter, (_total_steps_larger - _decel_steps), _total_steps_larger, _profile.getMaxRPM(), _profile.getMinRPM());
    }

    // Cruise Phase
    else {
        _current_rpm = _profile.getMaxRPM();
    }

    // Clamp RPM
    if (_current_rpm > 1200)
        _current_rpm = 1200;
    if (_current_rpm < 0)
        _current_rpm = 0;

    // Bresenham's line plotting algorithm
    int e2 = 2 * _err;
    if (e2 >= _d.y) {
        _err += _d.y;
        _left.stepHigh();
        _steps.x += (_left.getDir() ? 1 : -1);
    }
    if (e2 <= _d.x) {
        _err += _d.x;
        _right.stepHigh();
        _steps.y += (_right.getDir() ? 1 : -1);
    }

    // Update timer period / stop if motion ended
    if (getStepCount() < getTotalSteps()) {
        _timer.setPeriod(_step_period_from_rpm_over_two[_current_rpm]);
        // _timer.start();
    } else {
        _timer.stop();
    }
}

void Gantry::home() {
    // dummy function
}

void Gantry::pauseMotion() {
    _timer.stop();
}
