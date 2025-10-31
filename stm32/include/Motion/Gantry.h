#pragma once

#include "Motion/StepperDriver.h"
#include "Motion/Motor.h"
#include "Types/PinDef.h"

#include "Arduino.h"
#include <stdint.h>

class Gantry {
    private:
        static StepperDriver _l_driver, _r_driver;
        static PinDef *_miso, *_mosi, *_sclk;
        static Motor _l_motor, _r_motor;

    public:
        /**
         * @brief Initializes the gantry, its motors, and its pins
         */
        static void init();
};