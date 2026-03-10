#ifndef MOTOR_H
#define MOTOR_H

#include "Motion/Driver/HighPowerStepperDriver.h"
#include "Types/PinDef.h"

class Motor {
    private:
        HighPowerStepperDriver _driver;
        PinDef* _step;
        PinDef* _dir;
        PinDef* _scs;
        PinDef* _fault;

        /**
         * @brief Fault ISR responds to fault condition from motor driver;
         */
        void _faultISR();

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
        Motor(PinDef& step, PinDef& dir, PinDef& scs, PinDef& fault);

        /**
         * @brief Sets the motor's target direction
         * 
         * @param dir       The desired direction
         */
        void setDir(bool dir);

        /*
          * @return dir
         */
        uint8_t getDir() const { return _dir->read(); }

         /**
         * @brief Steps the motor in it's current direction
         */
        void step();

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

#endif
