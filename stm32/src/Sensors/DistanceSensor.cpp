#include "Sensors/DistanceSensor.h"

namespace {
    #define TRIG 0
    #define ECHO 1
}

double DistanceSensor::_speed_of_sound = 0.343;

DistanceSensor::DistanceSensor(PinDef& trig, PinDef& echo) : Sensor(trig, echo) {}

void DistanceSensor::calibrate(double temperature) {
    _speed_of_sound = 331.4e-3 + 0.6e-3 * temperature;
}

double DistanceSensor::distance() {
    _pins[TRIG]->write(LOW);
    delayMicroseconds(2);
    _pins[TRIG]->write(HIGH);
    delayMicroseconds(10);
    _pins[TRIG]->write(LOW);

    uint32_t duration = pulseIn(_pins[ECHO]->PIN, HIGH);
    // delayMicroseconds(100000);
    return 0.5 * duration * _speed_of_sound;
} 