#include "Motion/Motor.h"

Motor::Motor(PinDef& step, PinDef& dir, PinDef& scs, PinDef& fault) {
    _step = &step;
    _dir = &dir;
    _scs = &scs;
    _fault = &fault;
}

void Motor::init() {
    // Initialize pins
    _step->init();
    stepLow();
    _dir->init();
    _driver.setChipSelectPin(_scs->PIN);
    _fault->init();

    // attach fault interupt service routine
    attachInterrupt(_fault->PIN, [this](){_faultISR();}, FALLING);
    
    // Initialize driver (auto initializes SCS pin)
    delay(100);

    // _driver.resetSettings();
    // _driver.clearStatus();
    // _driver.setDecayMode(HPSDDecayMode::AutoMixed);
    // _driver.setCurrentMilliamps36v4(4000); 
    _driver.setStepMode();
    _driver.enableDriver();
}

void Motor::_faultISR() {
    // dummy function
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