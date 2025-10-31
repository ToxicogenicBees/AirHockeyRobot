#include "Motion/Motor.h"

Motor::Motor(StepperDriver* driver, PinDef& step, PinDef& dir, PinDef& scs) {
    _driver = driver;
    _step = &step;
    _dir = &dir;
    _scs = &scs;
}

void Motor::init() {
    
}