#pragma once

#include "Motion/StepperDriver.h"
#include "Types/PinDef.h"

class Motor {
    private:
        PinDef* _step, _dir, _scs;
        StepperDriver* _driver;

    public:
        friend class Gantry;

        /**
         * @brief Creates a new motor
         * 
         * @param driver    Pointer to stepper driver
         * @param step      Step pin
         * @param dir       Direction pin
         * @param scs       Chip select pin
         */
        Motor(StepperDriver* driver, PinDef& step, PinDef& dir, PinDef& scs);

        /**
         * @brief Initializes a motor and its pins
         */
        void init();
};