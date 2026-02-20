#include "Sensors/LimitSwitch.h"

LimitSwitch::LimitSwitch(PinDef& pin) : Sensor(&pin) {}

bool LimitSwitch::pressed() {
    return !_pins[0]->read();
}