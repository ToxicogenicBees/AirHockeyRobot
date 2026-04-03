#include "Types/PinDef.hpp"

PinDef::PinDef(uint8_t pin, uint8_t mode)
    : PIN(pin), MODE(mode) {}

uint8_t PinDef::read() const {
    return digitalRead(PIN) == HIGH;
}

uint32_t PinDef::readAnalog() const {
    return analogRead(PIN);
}

void PinDef::write(uint8_t state) {
    digitalWrite(PIN, state);
}

void PinDef::writeAnalog(uint32_t state) {
    analogWrite(PIN, state);
}

void PinDef::toggle() {
    digitalWrite(PIN, !digitalRead(PIN));
}

void PinDef::init() {
    pinMode(PIN, MODE);
}

PinDef::operator uint8_t() const {
    return read();
}
