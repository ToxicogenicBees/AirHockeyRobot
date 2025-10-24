#include "Types/PinDef.h"

PinDef::PinDef(uint8_t pin, uint8_t mode)
    : PIN(pin), MODE(mode) {}

bool PinDef::read() const {
    return digitalRead(PIN) == HIGH;
}

void PinDef::write(bool state) {
    digitalWrite(PIN, state ? HIGH : LOW);
}

void PinDef::toggle() {
    digitalWrite(PIN, !digitalRead(PIN));
}

void PinDef::init() {
    pinMode(PIN, MODE);
}

PinDef& PinDef::operator=(bool state) {
    write(state);
    return *this;
}

PinDef::operator bool() const {
    return read();
}