#include "Sensors/Sensor.h"
#include <stdarg.h>

Sensor::Sensor(uint8_t num_pins, ...): _NUM_PINS(num_pins) {
    _PINS = new Pin*[_NUM_PINS];

    va_list args;
    va_start(args, num_pins);

    for (uint8_t i = 0; i < _NUM_PINS; ++i) {
        _PINS[i] = va_arg(args, Pin*);
    }

    va_end(args);
}

Sensor::~Sensor() {
    delete[] _PINS;
}

void Sensor::init() {
    for (uint16_t i = 0; i < _NUM_PINS; i++)
        _PINS[i]->init();
}