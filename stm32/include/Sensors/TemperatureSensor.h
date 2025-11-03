#pragma once

#include "Sensors/Sensor.h"
#include "Types/PinDef.h"

#include <stdint.h>

class TemperatureSensor : public Sensor {
    public:
        /**
         * @brief Creates an temperature sensor
         * 
         * @param read ADC temperature read pin
         */
        TemperatureSensor(PinDef& read);

        /**
         * @brief Returns the temperature read by sensor in Celsius
         * 
         * @return Returns the temperature read by sensor in Celsius
         */
        float temperature();

    private:
        /**
         * @brief returns temperature in Celsius using 4th order regression from TI thermistor design tool
         * 
         * @param Vsense voltage sensed over thermistor
         */
        float transferFunctionTempVsVsense(float Vsense);
};