#include "Sensors/DistanceSensor.h"

DistanceSensor::DistanceSensor(PinDef& trig, PinDef& echo) : Sensor(2, &trig, &echo) {}

double DistanceSensor::distance() {
    _PINS[0]->write(false);
    delayMicroseconds(2);
    _PINS[0]->write(true);
    delayMicroseconds(10);
    _PINS[0]->write(false);

    uint32_t duration = pulseIn(_PINS[1]->PIN, HIGH);
    return duration * 0.1715; // 0.343 * 0.5
}