#include "Sensors/LimitSwitch.hpp"

LimitSwitch::LimitSwitch(PinDef& pin) : Sensor(pin) {}

bool LimitSwitch::pressed() {
    // return _pins[0]->readAnalog();  // if less than 250 mV count as pressed
    return !_pins[0]->read();
}
