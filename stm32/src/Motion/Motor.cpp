#include "Motion/Motor.h"

Motor::Motor(PinDef& step, PinDef& dir, PinDef& scs) {
    _step = &step;
    _dir = &dir;
    _scs = &scs;
}

void Motor::init() {
    // Initialize pins
    _step->init();
    _dir->init();
    
    // Initialize driver (auto initializes SCS pin)
    delay(10);

    _driver.setChipSelectPin(_scs->PIN);
    _driver.resetSettings();
    _driver.clearStatus();
    _driver.setDecayMode(HPSDDecayMode::AutoMixed);
    _driver.setCurrentMilliamps36v4(4000); 
    _driver.setStepMode((HPSDStepMode) MICROSTEP_SETTING);
    _driver.enableDriver();
}

void Motor::setDir(bool dir) {
    _dir->write(dir);
}

void Motor::step() {
    _step->write(HIGH);
    delayMicroseconds(2); 
    _step->write(LOW);
}

void Motor::stepHigh() {
    _step->write(HIGH);
}

void Motor::stepLow() {
    _step->write(LOW);
}