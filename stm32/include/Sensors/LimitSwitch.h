#pragma once

#include <stdint.h>
#include "Sensor.h"
#include "Pin.h"

class LimitSwitch : public Sensor {
    public:
        LimitSwitch(Pin& pin);

        bool pressed();

        operator bool();
};