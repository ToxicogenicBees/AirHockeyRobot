#include "Motion/Gantry.h"
#include "Pinout.h"

#include <functional>

const double Gantry::_DRIVE_PULLEY_RADIUS = 28; // 28 mm
const double Gantry::_STEP_CONVERSION_CONST = Motor::MICROSTEPS_PER_REV / (2*PI * _DRIVE_PULLEY_RADIUS);  // for converting delta X or Y to steps
const double Gantry::DIST_TOLERANCE_LOW = 2.5; // mm
const double Gantry::DIST_TOLERANCE_HIGH = 30; // mm
        
Motor Gantry::_left(motor_l_step, motor_l_dir, motor_l_scs);
Motor Gantry::_right(motor_r_step, motor_r_dir, motor_r_scs);

Point2<double> Gantry::_position;

Point2<double> Gantry::_current_target;
Point2<int> Gantry::_total_steps_to_target;
int Gantry::_total_steps_larger = 0;
int Gantry::_step_counter = 0;
int Gantry::_accel_steps = 0;
int Gantry::_decel_steps = 0;
double Gantry::_current_rpm = 0;
Point2<int> Gantry::_d = {0, 0};
int Gantry::_err = 0;
uint16_t Gantry::_current_period_us = 0;

VelocityProfile Gantry::_profile;

HardwareTimer* Gantry::_start_motors = nullptr; // initialized in Gantry::init
HardwareTimer* Gantry::_stop_motors = nullptr;  // initialized in Gantry::init

void Gantry::init() {
    // Initialize motors
    _right.init();
    delay(100);
    _left.init();

    // Initialize hardware timer for motor step signal
    // https://github.com/stm32duino/Arduino_Core_STM32/wiki/HardwareTimer-library
    auto init_timer = [](HardwareTimer*& hardware_timer, TIM_TypeDef* tim, std::function<void()> callback) {
        hardware_timer = new HardwareTimer(tim);  
        hardware_timer->setMode(1, TIMER_OUTPUT_DISABLED, 0);  // no pin output, only for interrupt
        hardware_timer->pause();
        hardware_timer->attachInterrupt(callback);
        hardware_timer->setOverflow(10000, MICROSEC_FORMAT); // 10000 microseconds
        hardware_timer->setCount(0);
    };

    init_timer(_start_motors, TIM3, Gantry::_stepMotion);
    init_timer(_stop_motors, TIM4, Gantry::_stopMotion);
}

void Gantry::setPosition(const Point2<double>& pos) {
    _position = pos;
}

void Gantry::setVelocityProfile(const VelocityProfile& profile) {
    _profile = profile;
}

double Gantry::_mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint16_t Gantry::_calculateStepPeriod(double rpm) {
    if (rpm <= 0)
        return std::numeric_limits<uint16_t>::max();

    // microsteps/sec = (RPM / 60) * steps_per_rev
    double microsteps_per_second = (rpm / 60.0) * Motor::MICROSTEPS_PER_REV;
    return (uint16_t)(1000000.0 / microsteps_per_second);
}

Point2<int> Gantry::_calculateSteps(const Point2<double>& target) {
    auto step_displacement = _STEP_CONVERSION_CONST * (target - _position);

    return {
        (int) (step_displacement.x + step_displacement.y),
        (int) (step_displacement.x - step_displacement.y)
    };
}

void Gantry::initMotion(const Point2<double>& target) {
    _current_target = target;
    _total_steps_to_target = _calculateSteps(_current_target);
    _total_steps_larger = abs(_total_steps_to_target.x) > abs(_total_steps_to_target.y) ? abs(_total_steps_to_target.x) : abs(_total_steps_to_target.y);
    _step_counter = 0;

    _left.setDir(_total_steps_to_target.x < 0);
    _right.setDir(_total_steps_to_target.y < 0);

    _accel_steps = _total_steps_larger * _profile.getAccelPercent();
    _decel_steps = _total_steps_larger * _profile.getDecelPercent();
    
    // Ensure cruise phase exists
    if (_accel_steps + _decel_steps > _total_steps_larger) {
        _accel_steps = _total_steps_larger / 2;
        _decel_steps = _total_steps_larger / 2;
    }

    // Calculate errors
    _d = {abs(_total_steps_to_target.x), -abs(_total_steps_to_target.y)};
    _err = _d.x + _d.y;
}

void Gantry::startMotion() {
    // To check if paused:
    if (_start_motors->getCount() == 0) {
        _stepMotion();
    }
}

void Gantry::_stepMotion() {
    ++_step_counter;
    _current_period_us = _calculateStepPeriod(_current_rpm);

    if (_step_counter < _accel_steps) {
        _current_rpm = _mapDouble(_step_counter, 0, _accel_steps, _profile.getMinRPM(), _profile.getMaxRPM());
    } 
    // 2. Deceleration Phase
    else if (_step_counter >= (_total_steps_larger - _decel_steps)) {
        // Map from end of move back down to min speed
        _current_rpm = _mapDouble(_step_counter, (_total_steps_larger - _decel_steps), _total_steps_larger, _profile.getMaxRPM(), _profile.getMinRPM());
    } 
    // 3. Cruise Phase
    else {
        _current_rpm = _profile.getMaxRPM();
    }

    // Bresenham's line plotting algorithm
    double e2 = 2 * _err;
    double d_a = 0;
    double d_b = 0;

    auto step = [](Motor& motor) {
        motor.stepHigh();
        return (motor.getDir() ? -1 : 1) * 2 * PI / Motor::MICROSTEPS_PER_REV * _DRIVE_PULLEY_RADIUS;
    };

    if (e2 >= _d.y) {
        _err += _d.y;
        d_a = step(_left);
    } /* e_xy+e_x > 0 */
        
    if (e2 <= _d.x) {
        _err += _d.x;
        d_b = step(_right);
    } /* e_xy+e_y < 0 */

    _start_motors->pause();

    _stop_motors->setOverflow(2, MICROSEC_FORMAT);
    _stop_motors->setCount(0);
    _stop_motors->resume();

    // update current assumed position (open-loop control)
    _position.x += 0.5 * (d_a + d_b);
    _position.y += 0.5 * (d_a - d_b);
}

void Gantry::_stopMotion() {
    _left.stepLow();
    _right.stepLow();

    _stop_motors->pause();
    _stop_motors->setCount(0);

    if (getStepCount() < getTotalSteps()) { // set timer to trigger next step after _current_period_us
        _start_motors->setOverflow(_current_period_us, MICROSEC_FORMAT);
        _start_motors->setCount(0);
        _start_motors->resume();
    } else {
        _start_motors->pause();
        _start_motors->setCount(0);
        _stop_motors->pause();
        _stop_motors->setCount(0);
    }
}

void Gantry::home() {
    // dummy function
}
