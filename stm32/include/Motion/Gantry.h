#pragma once

#include "Motion/Driver/HighPowerStepperDriver.h"
#include "Types/Point2.hpp"
#include "Motion/Motor.h"
#include "Types/PinDef.h"
#include "Pinout.h"

#include "Arduino.h"
#include <stdint.h>
#include <numeric>

class Gantry {
    private:
        static Motor _left, _right;

    public:
        /**
         * @brief Initializes the gantry, its motors, and its pins
         */
        static void init();

        /**
         * @brief Updates the target position of the gantry
         */
        static void setTarget(const Point2<double>& target);
};