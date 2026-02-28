#include "Motion/Gantry.h"
#include "Pinout.h"

const float Gantry::_DRIVE_PULLEY_RADIUS = 28; // 28 mm
const float Gantry::_STEP_CONVERSION_CONST = Motor::MICROSTEPS_PER_REV / (2*PI * _DRIVE_PULLEY_RADIUS);  // for converting delta X or Y to steps
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

double Gantry::_accel_percent;
double Gantry::_decel_percent;
double Gantry::_min_rpm;
double Gantry::_max_rpm;

HardwareTimer *Gantry::_increment_straight_line_movement_timer = nullptr;   // initialized in Gantry::init
HardwareTimer *Gantry::_pull_down_motor_step_pins_timer = nullptr;  // initialized in Gantry::init

void Gantry::init() {
    // Initialize motors
    _right.init();
    delay(100);
    _left.init();

    // Initializze hardware timer for motor step signal
    // https://github.com/stm32duino/Arduino_Core_STM32/wiki/HardwareTimer-library
    _increment_straight_line_movement_timer = new HardwareTimer(TIM3);  
    _increment_straight_line_movement_timer->setMode(1, TIMER_OUTPUT_DISABLED, 0);  // no pin output, only for interrupt
    _increment_straight_line_movement_timer->pause();
    _increment_straight_line_movement_timer->attachInterrupt(Gantry::_incrementStraightLineMovement);
    _increment_straight_line_movement_timer->setOverflow(10000, MICROSEC_FORMAT); // 10000 microseconds
    _increment_straight_line_movement_timer->setCount(0);

    _pull_down_motor_step_pins_timer = new HardwareTimer(TIM4);  
    _pull_down_motor_step_pins_timer->setMode(1, TIMER_OUTPUT_DISABLED, 0);  // no pin output, only for interrupt
    _pull_down_motor_step_pins_timer->pause();
    _pull_down_motor_step_pins_timer->attachInterrupt(Gantry::_pullDownMotorStepPinsAndRestartIncrementTimer);
    _pull_down_motor_step_pins_timer->setOverflow(10000, MICROSEC_FORMAT); // 10000 microseconds
    _pull_down_motor_step_pins_timer->setCount(0);
}

void Gantry::setPosition(const Point2<double>& pos) {
    _position = pos;
}

void Gantry::setVelocityProfile(double min_rpm, double max_rpm, double accel_percent, double decel_percent) {
    _min_rpm = min_rpm;
    _max_rpm = max_rpm;
    _accel_percent = accel_percent;
    _decel_percent = decel_percent;
}

float Gantry::_mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint16_t Gantry::_calculateStepPeriod(double rpm) {
    if (rpm <= 0)
        return 65535;
    // microsteps/sec = (RPM / 60) * steps_per_rev
    double microstepsPerSecond = (rpm / 60.0) * Motor::MICROSTEPS_PER_REV;
    return (uint16_t)(1000000.0 / microstepsPerSecond);
}

Point2<int> Gantry::_calculateMotorSteps(const Point2<double>& target) {
    float dx = target.x - _position.x;
    float dy = target.y - _position.y;

    // // first calculate steps for x movement 
    // // (common-mode input with both motors stepping same direction)
    // float sax = _STEP_CONVERSION_CONST * dx;
    // float sbx = sax;

    // // then calculate steps for y movement
    // // (differntial mode input with the motors stepping in opposite directions)
    // float say = _STEP_CONVERSION_CONST * dy;
    // float sby = -say;

    // // finally superimpose the x and y movements by summing the steps
    // float sa = sax + say;
    // float sb = sbx + sby;

    // return {(int)sa, (int)sb};

    return {
        (int) (_STEP_CONVERSION_CONST * (dx + dy)),
        (int) (_STEP_CONVERSION_CONST * (dx - dy))
    };
}

void Gantry::setUpStraightLineMovement(const Point2<double>& target) {
    _current_target = target;
    _total_steps_to_target = _calculateMotorSteps(_current_target);
    _total_steps_larger = abs(_total_steps_to_target.x) > abs(_total_steps_to_target.y) ? abs(_total_steps_to_target.x) : abs(_total_steps_to_target.y);
    _step_counter = 0;

    _left.setDir(_total_steps_to_target.x < 0);
    _right.setDir(_total_steps_to_target.y < 0);

    _accel_steps = _total_steps_larger * _accel_percent;
    _decel_steps = _total_steps_larger * _decel_percent;
    
    // Ensure cruise phase exists
    if (_accel_steps + _decel_steps > _total_steps_larger) {
        _accel_steps = _total_steps_larger / 2;
        _decel_steps = _total_steps_larger / 2;
    }

    _d = {abs(_total_steps_to_target.x), -abs(_total_steps_to_target.y)};
    _err = _d.x + _d.y;
}

void Gantry::startOrContiueStraightLineMovement() {
    // To check if paused:
    if (_increment_straight_line_movement_timer->getCount() == 0) {
        _incrementStraightLineMovement();
    }
}

void Gantry::_incrementStraightLineMovement() {
    ++_step_counter;
    _current_period_us = _calculateStepPeriod(_current_rpm);

    if (_step_counter < _accel_steps) {
        _current_rpm = _mapFloat(_step_counter, 0, _accel_steps, _min_rpm, _max_rpm);
    } 
    // 2. Deceleration Phase
    else if (_step_counter >= (_total_steps_larger - _decel_steps)) {
        // Map from end of move back down to min speed
        _current_rpm = _mapFloat(_step_counter, (_total_steps_larger - _decel_steps), _total_steps_larger, _max_rpm, _min_rpm);
    } 
    // 3. Cruise Phase
    else {
        _current_rpm = _max_rpm;
    }

    // Bresenham's line plotting algorithm
    double e2 = 2 * _err;
    double dA = 0;
    double dB = 0;

    if (e2 >= _d.y) {
        _err += _d.y;
        _left.stepHigh();
        dA = (_left.getDir() ? -1 : 1) * 2*PI/Motor::MICROSTEPS_PER_REV * _DRIVE_PULLEY_RADIUS;
    } /* e_xy+e_x > 0 */
        
    if (e2 <= _d.x) {
        _err += _d.x;
        _right.stepHigh();
        dB = (_right.getDir() ? -1 : 1) * 2*PI/Motor::MICROSTEPS_PER_REV * _DRIVE_PULLEY_RADIUS;
    } /* e_xy+e_y < 0 */

    _increment_straight_line_movement_timer->pause();

    _pull_down_motor_step_pins_timer->setOverflow(2, MICROSEC_FORMAT);
    _pull_down_motor_step_pins_timer->setCount(0);
    _pull_down_motor_step_pins_timer->resume();

    // update current assumed position (open-loop control)
    _position.x += 0.5 * (dA + dB);
    _position.y += 0.5 * (dA - dB);
}

void Gantry::_pullDownMotorStepPinsAndRestartIncrementTimer() {
    _left.stepLow();
    _right.stepLow();

    _pull_down_motor_step_pins_timer->pause();
    _pull_down_motor_step_pins_timer->setCount(0);

    if (getStepCount() < getTotalSteps()) { // set timer to trigger next step after _current_period_us
        _increment_straight_line_movement_timer->setOverflow(_current_period_us, MICROSEC_FORMAT);
        _increment_straight_line_movement_timer->setCount(0);
        _increment_straight_line_movement_timer->resume();
    } else {
        _increment_straight_line_movement_timer->pause();
        _increment_straight_line_movement_timer->setCount(0);
        _pull_down_motor_step_pins_timer->pause();
        _pull_down_motor_step_pins_timer->setCount(0);
    }
}

void Gantry::runHomingRoutine() {
    // dummy function
}