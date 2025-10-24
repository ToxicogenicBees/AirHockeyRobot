#include "Sensors/DistanceSensor.h"

void DistanceSensor::_onInterrupt() {
    if (_PINS[1]->read())
        _cur_dist = 0.1715 * (_update_time - HAL_GetTick());
}

DistanceSensor::DistanceSensor(Pin& trig, Pin& echo) : Sensor(2, trig, echo) {}

void DistanceSensor::update() {
    _update_time = HAL_GetTick();

    _PINS[0]->write(true);
    HAL_Delay(1);
    _PINS[0]->write(false);
}