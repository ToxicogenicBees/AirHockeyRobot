#include "Sensors/EmergencyStop.hpp"
#include "Comms/SerialLink.hpp"

EmergencyStop::EmergencyStop(PinDef& pin)
    : Sensor(pin) {}

void EmergencyStop::update() {
    // Get the pin state
    auto pressed = !_pins[0]->read();

    // Update enabled flag
    if ((pressed && !_enabled) || (!pressed && _enabled)) {
        _enabled = !_enabled;
    }

    // Send enabled state to laptop
    Packet e_stop_triggered(Action::EStop);
    e_stop_triggered << _enabled;
    SerialLink::buffer(e_stop_triggered);
}

bool EmergencyStop::enabled() const {
    return _enabled;
}