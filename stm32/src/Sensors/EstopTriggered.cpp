#include "Sensors/EstopTriggered.hpp"

EstopTriggered::EstopTriggered(PinDef& pin) : Sensor(pin) {
    attachInterrupt(pin, [this](){_estopTriggeredISR();}, FALLING);
}

bool EstopTriggered::pressed() {
    return !_pins[0]->read();
}

void EstopTriggered::_estopTriggeredISR() {
    // dummy function
}