#ifndef LIMITSWITCH_H
#define LIMITSWITCH_H

#include "Sensors/Sensor.hpp"
#include "Types/PinDef.h"

#include <stdint.h>

class LimitSwitch : public Sensor {
    public:
        /**
         * @brief Create a new limit switch
         * 
         * @param pin   The limit switch's pin
         */
        LimitSwitch(PinDef& pin);

        /**
         * @brief Returns if the limit switch is pressed
         *        Assumes HIGH voltage signal when pressed
         * 
         * @return The state of the limit switch
         */
        bool pressed();
};

#endif
