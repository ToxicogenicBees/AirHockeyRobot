#include "Pin.h"

Pin::Pin(uint32_t pin, uint32_t mode)
    : PIN(pin), MODE(mode) {}

bool Pin::read() const {
    return digitalRead(PIN) == HIGH;
}

void Pin::write(bool state) {
    digitalWrite(PIN, state ? HIGH : LOW);
}

void Pin::toggle() {
    digitalWrite(PIN, !digitalRead(PIN));
}

void Pin::init() {
    pinMode(PIN, MODE);
}

Pin& Pin::operator=(bool state) {
    write(state);
    return *this;
}

Pin::operator bool() const {
    return read();
}