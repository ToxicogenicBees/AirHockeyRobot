#pragma once

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "Pin.h"

class Sensor {
    protected:
        static Sensor* _instances[16];

        const uint8_t _NUM_PINS;
        Pin** _PINS;

        void _registerInterrupt(Pin& pin);
        virtual void _onInterrupt() {}

    public:
        static void handleInterrupt(uint16_t GPIO_Pin);

        Sensor(uint8_t num_pins, ...);
        
        virtual ~Sensor();

        virtual void init();
};