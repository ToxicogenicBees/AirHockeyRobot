#include "Sensors/DistanceSensor.h"

namespace {
    #define TRIG 0
    #define ECHO 1
}

double DistanceSensor::_speed_of_sound = 0.343;

DistanceSensor::DistanceSensor(PinDef& trig, PinDef& echo) : Sensor(2, &trig, &echo) {}

void DistanceSensor::calibrate(double temperature) {
    _speed_of_sound = 331.4e-3 + 0.6e-3 * temperature;
}

double DistanceSensor::distance() {
    _PINS[TRIG]->write(false);
    delayMicroseconds(2);
    _PINS[TRIG]->write(true);
    delayMicroseconds(10);
    _PINS[TRIG]->write(false);

    uint32_t duration = pulseIn(_PINS[ECHO]->PIN, HIGH);
    return 0.5 * duration * _speed_of_sound;
}