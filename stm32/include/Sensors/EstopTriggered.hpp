#pragma once

#include "Sensors/Sensor.hpp"
#include "Types/PinDef.hpp"

#include <stdint.h>

class EstopTriggered : public Sensor {
    private:
        /**
         * @brief ISR triggered when estop pressed
         */
        void _estopTriggeredISR();
    public:
        /**
         * @brief Create a new estop triggered
         * 
         * @param pin   The estop's pin
         */
        EstopTriggered(PinDef& pin);

        /**
         * @brief Returns if the estop switch is pressed
         *        Assumes LOW voltage signal when pressed
         * 
         * @return The state of the estop switch
         */
        bool pressed();
};
