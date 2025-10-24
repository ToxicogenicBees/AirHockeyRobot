#include "Sensors/LimitSwitch.h"

LimitSwitch::LimitSwitch(Pin& pin) : Sensor(1, &pin) {}

bool LimitSwitch::pressed() {
    return _PINS[0]->read();
}

LimitSwitch::operator bool() {
    return _PINS[0]->read();
}