#include "Types/PinDef.h"

PinDef::PinDef(uint8_t pin, uint8_t mode)
    : PIN(pin), MODE(mode) {}

bool PinDef::read() const {
    return digitalRead(PIN) == HIGH;
}

uint32_t PinDef::readAnalog() const {
    return analogRead(PIN);
}

void PinDef::write(bool state) {
    digitalWrite(PIN, state ? HIGH : LOW);
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

PinDef::operator bool() const {
    return read();
}