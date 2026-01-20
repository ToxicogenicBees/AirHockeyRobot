#pragma once

#include "Motion/Driver/HighPowerStepperDriver.h"
#include "Types/PinDef.h"

class Motor {
    private:
        HighPowerStepperDriver _driver;     // Motor driver
        PinDef *_step, *_dir, *_scs;        // Motor pins

    public:
        friend class Gantry;

        // Motor & ramp config
        static const uint16_t FULL_STEPS_PER_REV = 200;
        static const uint16_t MICROSTEP_SETTING = 8;
        static const uint32_t MICROSTEPS_PER_REV = (uint32_t)FULL_STEPS_PER_REV * (uint32_t)MICROSTEP_SETTING;

        // Motion config
        static const float ACCEL_PERCENT = 0.20;    // X% of travel is acceleration
        static const float DECEL_PERCENT = 0.20;    // X% of travel is deceleration
        static const float MAX_RPM = 150.0;         // Top speed
        static const float MIN_RPM = 30.0;          // Starting speed

        /**
         * @brief Creates a new motor
         * 
         * @param step      Step pin
         * @param dir       Direction pin
         * @param scs       Chip select pin
         */
        Motor(PinDef& step, PinDef& dir, PinDef& scs);

        /**
         * @brief Sets the motor's target direction
         * 
         * @param dir       The desired direction
         */
        void setDir(bool dir);

         /**
         * @brief Steps the motor in it's current direction
         */
        void step();

        /**
         * @brief Initializes a motor and its pins
         */
        void init();
};