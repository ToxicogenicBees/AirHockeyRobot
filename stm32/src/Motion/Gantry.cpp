#include "Motion/Gantry.h"

Motor Gantry::_left(motor_l_step, motor_l_dir, motor_l_scs);
Motor Gantry::_right(motor_r_step, motor_r_dir, motor_r_scs);

void Gantry::init() {
    // Initialize motors
    _left.init();
    _right.init();
}