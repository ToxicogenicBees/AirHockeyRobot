#include "Sensors/DistanceSensor.h"

namespace {
    #define TRIG 0
    #define ECHO 1
}

DistanceSensor::DistanceSensor(PinDef& trig, PinDef& echo) : Sensor(2, &trig, &echo) {}

double DistanceSensor::distance() {
    _PINS[TRIG]->write(false);
    delayMicroseconds(2);
    _PINS[TRIG]->write(true);
    delayMicroseconds(10);
    _PINS[TRIG]->write(false);

    uint32_t duration = pulseIn(_PINS[ECHO]->PIN, HIGH);
    return duration * 0.1715; // 0.343 * 0.5
}