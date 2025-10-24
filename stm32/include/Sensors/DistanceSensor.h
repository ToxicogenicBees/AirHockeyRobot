#pragma once

#include <stdint.h>
#include "Sensor.h"
#include "Pin.h"

class DistanceSensor : public Sensor {
    public:
        DistanceSensor(Pin& trig, Pin& echo);

        double distance();
};