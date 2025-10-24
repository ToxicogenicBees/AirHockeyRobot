#pragma once

#include <stdint.h>
#include "Pin.h"

class Sensor {
    protected:
        const uint8_t _NUM_PINS;
        Pin** _PINS;

    public:
        Sensor(uint8_t num_pins, ...);
        
        virtual ~Sensor();

        virtual void init();
};