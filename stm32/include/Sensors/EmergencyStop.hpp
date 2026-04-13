#pragma once

#include "Sensors/Sensor.hpp"
#include "Types/PinDef.hpp"

#include <stdint.h>

class EmergencyStop : public Sensor {
    private:
        bool _enabled = false;  // Enabled flag

    public:
        /**
         * @brief Create a new e-stop
         * 
         * @param pin   The e-stop's pin
         */
        EmergencyStop(PinDef& pin);

        /**
         * @brief Reads and updates the e-stop's state
         */
        void update();

        /**
         * @brief Returns if the e-stop switch is pressed
         *        Assumes LOW voltage signal when pressed
         * 
         * @return The state of the e-stop switch
         */
        bool enabled() const;
};
