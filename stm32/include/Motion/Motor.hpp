#pragma once

#include "Motion/Driver/HighPowerStepperDriver.hpp"
#include "Types/PinDef.hpp"

class Motor {
    private:
        HighPowerStepperDriver _driver;
        PinDef* _step;
        PinDef* _dir;
        PinDef* _scs;
        PinDef* _fault;
        PinDef* _sleep;
        PinDef* _enable;

    public:
        // Motor config
        static const uint16_t FULL_STEPS_PER_REV = 200;
        static const uint16_t MICROSTEP_SETTING = 8;
        static const uint32_t MICROSTEPS_PER_REV = (uint32_t)FULL_STEPS_PER_REV * (uint32_t)MICROSTEP_SETTING;

        /**
         * @brief Creates a new motor
         * 
         * @param step      Step pin
         * @param dir       Direction pin
         * @param scs       Chip select pin
         * @param fault       Chip select pin
         */
        Motor(PinDef& step, PinDef& dir, PinDef& scs, PinDef& fault, PinDef& sleep, PinDef& enable);

        /**
         * @brief Sets the motor's target direction
         * 
         * @param dir       The desired direction
         */
        void setDir(uint8_t dir);

        /**
         * @brief Get the motor's direction pin state
         * 
         * @return The motor's direction pin state
         */
        uint8_t getDir() const { return _dir->read(); }

         /**
         * @brief Sets step pin high;
         */
        void stepHigh();

         /**
         * @brief Sets step pin low;
         */
        void stepLow();

        /**
         * @brief Initializes a motor and its pins
         */
        void init();
};
