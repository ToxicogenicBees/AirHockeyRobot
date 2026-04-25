#include "Motion/Motor.hpp"

Motor::Motor(PinDef& step, PinDef& dir, PinDef& scs, PinDef& fault, PinDef& sleep, PinDef& enable) {
    _step = &step;
    _dir = &dir;
    _scs = &scs;
    _fault = &fault;
    _sleep = &sleep;
    _enable = &enable;
}

void Motor::init() {
    // Initialize pins
    _step->init();
    stepLow();
    _dir->init();
    _driver.setChipSelectPin(_scs->PIN);
    _fault->init();
    _sleep->init();
    _enable->init();

    _sleep->write(HIGH);
    delay(1);
    _enable->write(HIGH);
    
    // Initialize driver (auto initializes SCS pin)
    delay(100);

    _driver.setCurrentHold(0x01);
    _driver.enableStandstillPowerSavingMode();
    _driver.setStepMode();
    _driver.enableDriver();
}

void Motor::setDir(uint8_t dir) {
    _dir->write(dir);
}

void Motor::stepHigh() {
    _step->write(HIGH);
}

void Motor::stepLow() {
    _step->write(LOW);
}
