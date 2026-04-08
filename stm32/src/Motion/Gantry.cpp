#include "Motion/Gantry.hpp"
#include "Pinout.hpp"
#include "Constants.hpp"

#include <functional>

namespace {
    #ifndef step_intermission_timer
        #define step_intermission_timer TIM3
    #endif

    #ifndef step_period_timer
        #define step_period_timer TIM2
    #endif

    constexpr double DRIVE_PULLEY_RADIUS = 28.0; // 28 mm
    constexpr double STEP_CONVERSION_CONST = Motor::MICROSTEPS_PER_REV / (2*PI * DRIVE_PULLEY_RADIUS);  // for converting delta X or Y to steps

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

HardwareTimer* Gantry::_step_period_timer = nullptr; // initialized in Gantry::init
HardwareTimer* Gantry::_step_intermission_timer = nullptr;  // initialized in Gantry::init

uint16_t Gantry::_step_period_from_rpm_over_two[1201];   // max rpm is 1200

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
        hardware_timer->setCount(0);
        hardware_timer->setPrescaleFactor((hardware_timer->getTimerClkFreq() / 1000000) - 1);
    };

    init_timer(_step_period_timer, step_period_timer, _stepMotion);

    // step_period_timer->CR1 |= TIM_CR1_CEN;
    // step_period_timer->CR1 &= ~TIM_CR1_ARPE;

    // populate rpm step period lookup table
    for (int i = 0; i <= 1200; ++i) {
        _step_period_from_rpm_over_two[i] = _calculateStepPeriod(i) / 2;
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

int Gantry::_mapInt(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint16_t Gantry::_calculateStepPeriod(int rpm) {
    if (rpm <= 0)
        return std::numeric_limits<uint16_t>::max();

    // microsteps/sec = (RPM / 60) * steps_per_rev
    double microsteps_per_second = (rpm / 60.0) * Motor::MICROSTEPS_PER_REV;
    return (uint16_t)(1e6 / microsteps_per_second);
}

Point2<int> Gantry::_calculateSteps(const Point2<double>& target) {
    auto step_displacement = STEP_CONVERSION_CONST * (target - getPosition());

    return {
        (int) (step_displacement.x + step_displacement.y),
        (int) (step_displacement.x - step_displacement.y)
    };
}

void Gantry::initMotion(const Point2<double>& target) {
    _current_target = target;
    _total_steps_to_target = _calculateSteps(_current_target);
    _total_steps_larger = std::abs(_total_steps_to_target.x) > std::abs(_total_steps_to_target.y)
        ? std::abs(_total_steps_to_target.x)
        : std::abs(_total_steps_to_target.y);
    _step_counter = 0;

    _left.setDir(_total_steps_to_target.x > 0);
    _right.setDir(_total_steps_to_target.y > 0);

    _accel_steps = _total_steps_larger * _profile.getAccelPercent();
    _decel_steps = _total_steps_larger * _profile.getDecelPercent();
    
    // Ensure cruise phase exists
    if (_accel_steps + _decel_steps > _total_steps_larger) {
        _accel_steps = _total_steps_larger / 2;
        _decel_steps = _total_steps_larger / 2;
    }

    // Calculate errors
    _d = {std::abs(_total_steps_to_target.x), -std::abs(_total_steps_to_target.y)};
    _err = _d.x + _d.y;
}

void Gantry::startMotion() {
    // To check if paused:
    if (_step_period_timer->getCount() == 0) {
        _stepMotion();
    }
}

int count = 0;
void Gantry::_stepMotion() {
    if (++count % 2) {
        _left.stepLow();
        _right.stepLow();
        
        return;
    }

    ++_step_counter;

    if (_step_counter < _accel_steps) {
        _current_rpm = _mapInt(_step_counter, 0, _accel_steps, _profile.getMinRPM(), _profile.getMaxRPM());
    } 
    // 2. Deceleration Phase
    else if (_step_counter >= (_total_steps_larger - _decel_steps)) {
        // Map from end of move back down to min speed
        _current_rpm = _mapInt(_step_counter, (_total_steps_larger - _decel_steps), _total_steps_larger, _profile.getMaxRPM(), _profile.getMinRPM());
    } 
    // 3. Cruise Phase
    else {
        _current_rpm = _profile.getMaxRPM();
    }

    // _current_period_us = _calculateStepPeriod(_current_rpm);
    // _current_period_us = _step_period_from_rpm_over_two[_current_rpm];

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

    // step_period_timer->CR1 &= ~TIM_CR1_CEN;
    if (getStepCount() < getTotalSteps()) {
        // step_period_timer->CR1 &= ~TIM_CR1_CEN;
        step_period_timer->ARR = _step_period_from_rpm_over_two[_current_rpm];
        step_period_timer->CNT = 0;
        step_period_timer->CR1 |= TIM_CR1_CEN;
    } else {
        step_period_timer->CR1 &= ~TIM_CR1_CEN;
        step_period_timer->CNT = 0;
    }
}

void Gantry::_stepIntermission() {
    _left.stepLow();
    _right.stepLow();

    step_intermission_timer->CR1 &= ~TIM_CR1_CEN;
    step_intermission_timer->CNT = 0;

    if (getStepCount() < getTotalSteps()) { // set timer to trigger next step after _current_period_us
        step_period_timer->ARR = _current_period_us;
        step_period_timer->CNT = 0;
        step_period_timer->CR1 |= TIM_CR1_CEN;
    } else {
        step_period_timer->CR1 &= ~TIM_CR1_CEN;
        step_period_timer->CNT = 0;

        step_intermission_timer->CR1 &= ~TIM_CR1_CEN;
        step_intermission_timer->CNT = 0;
    }
}

void Gantry::home() {
    // dummy function
}

void Gantry::pauseMotion() {
    step_period_timer->CR1 &= ~TIM_CR1_CEN;
    step_period_timer->CNT = 0;

    step_intermission_timer->CR1 &= ~TIM_CR1_CEN;
    step_intermission_timer->CNT = 0;
}
