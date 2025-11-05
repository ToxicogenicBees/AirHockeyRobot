#include "Sensors/LimitSwitch.h"

LimitSwitch::LimitSwitch(PinDef& pin) : Sensor(1, &pin) {}

bool LimitSwitch::pressed() {
    return !_PINS[0]->read();
}