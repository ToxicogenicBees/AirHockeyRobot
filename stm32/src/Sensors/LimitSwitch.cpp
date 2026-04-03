#include "Sensors/LimitSwitch.hpp"

LimitSwitch::LimitSwitch(PinDef& pin) : Sensor(pin) {}

bool LimitSwitch::pressed() {
    return !_pins[0]->read();
}
