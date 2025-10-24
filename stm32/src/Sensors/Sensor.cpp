#include "Sensors/Sensor.h"
#include <stdarg.h>

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    Sensor::handleInterrupt(GPIO_Pin);
}

Sensor* Sensor::_instances[16] = {nullptr};

void Sensor::_registerInterrupt(Pin& GPIO_Pin) {
    _instances[__builtin_ctz(GPIO_Pin)] = this;
}

void Sensor::handleInterrupt(uint16_t GPIO_Pin) {
    _instances[__builtin_ctz(GPIO_Pin)]->_onInterrupt();
}

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