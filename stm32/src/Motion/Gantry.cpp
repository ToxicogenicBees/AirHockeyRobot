#include "Motion/Gantry.h"

const float Gantry::_DRIVE_PULLEY_RADIUS = 28; // 28 mm
const float Gantry::_STEP_CONVERSION_CONST = Motor::MICROSTEPS_PER_REV / (2*PI * _DRIVE_PULLEY_RADIUS);  // for converting delta X or Y to steps

Motor Gantry::_left(motor_l_step, motor_l_dir, motor_l_scs);
Motor Gantry::_right(motor_r_step, motor_r_dir, motor_r_scs);

Point2<double> Gantry::_position;

double Gantry::_accel_percent;
double Gantry::_decel_percent;
double Gantry::_min_rpm;
double Gantry::_max_rpm;


void Gantry::init() {
    // Initialize motors
    _left.init();
    _right.init();
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

    // first calculate steps for x movement 
    // (common-mode input with both motors stepping same direction)
    float sax = _STEP_CONVERSION_CONST * dx;
    float sbx = sax;

    // then calculate steps for y movement
    // (differntial mode input with the motors stepping in opposite directions)
    float say = _STEP_CONVERSION_CONST * dy;
    float sby = -say;

    // finally superimpose the x and y movements by summing the steps
    float sa = sax + say;
    float sb = sbx + sby;

    return {(int)sa, (int)sb};
}

void Gantry::_runStraighLine(int steps_a, int steps_b) {
    uint32_t total_steps_larger = steps_a > steps_b ? steps_a : steps_b;
    uint32_t accel_steps = total_steps_larger * _accel_percent;
    uint32_t decel_steps = total_steps_larger * _decel_percent;
    
    // Ensure cruise phase exists
    if (accel_steps + decel_steps > total_steps_larger) {
        accel_steps = total_steps_larger / 2;
        decel_steps = total_steps_larger / 2;
    }

    float currentRpm;

    int dx = steps_a;
    int dy = -steps_b;
    int err = dx + dy;
    int e2 = 0;
    int steps_completed_a = 0;
    int steps_completed_b = 0;
    bool stepA = false; // if equals 1 then step both
    bool stepB = false; // if equals 1 then step both

    for (int i = 0; i < total_steps_larger; i++) {
        if (abs(steps_completed_a) >= abs(steps_a) || abs(steps_completed_b) >= abs(steps_b))
            break;

        // 1. Acceleration Phase
        if (i < accel_steps) {
            currentRpm = _mapFloat(i, 0, accel_steps, _min_rpm, _max_rpm);
        } 
        // 2. Deceleration Phase
        else if (i >= (total_steps_larger - decel_steps)) {
            // Map from end of move back down to min speed
            currentRpm = _mapFloat(i, (total_steps_larger - decel_steps), total_steps_larger, _max_rpm, _min_rpm);
        } 
        // 3. Cruise Phase
        else {
            currentRpm = _max_rpm;
        }

        uint16_t currentPeriodUs = _calculateStepPeriod(currentRpm);
        /*    
            // --- Safety Check ---
            if (digitalRead(LimitXMinPin) == LOW || digitalRead(LimitXMaxPin) == LOW ||
                digitalRead(LimitYMinPin) == LOW || digitalRead(LimitYMaxPin) == LOW) {
                Serial.println("!!! LIMIT HIT !!!");
                return; 
            }
        */

        e2 = 2*err;

        if (e2 >= dy) {
            err += dy;
            steps_completed_a++;
            _left.stepHigh();
        } /* e_xy+e_x > 0 */
            
        if (e2 <= dx) {
            err += dx;
            steps_completed_b++;
            _right.stepHigh();
        } /* e_xy+e_y < 0 */

        delayMicroseconds(2);
        _left.stepLow();
        _right.stepLow();

        delayMicroseconds(currentPeriodUs);
    }
}

void Gantry::goToPointInStraightLine(const Point2<double>& target) {
    Point2<int> steps = _calculateMotorSteps(target);

    _left.setDir(steps.x < 0);
    _right.setDir(steps.y > 0);

    _runStraighLine(steps.x, steps.y);
}