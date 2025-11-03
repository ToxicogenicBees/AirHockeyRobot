#pragma once

#include "Sensors/Sensor.h"
#include "Types/PinDef.h"

#include <stdint.h>

class TemperatureSensor : public Sensor {
    private:
        // TMP6131LPGM thermistor
        // Transfer function for temperature vs. Vsense voltage
        // is given in the TI Thermistor design tool
        static const float _THRM_A0 = -2.885698e2;
        static const float _THRM_A1 = 1.556236e2;
        static const float _THRM_A2 = 7.191258e1;
        static const float _THRM_A3 = -5.134061e1;
        static const float _THRM_A4 = 1.244030e1;

        /**
         * @brief Returns temperature in Celsius using 4th order regression from TI thermistor design tool
         * 
         * @param Vsense Voltage sensed over thermistor
         */
        float _transferFunctionTempVsVsense(float v_sense);

    public:
        /**
         * @brief Creates a temperature sensor
         * 
         * @param read ADC temperature read pin
         */
        TemperatureSensor(PinDef& read);

        /**
         * @brief Returns the temperature read by sensor in Celsius
         * 
         * @return Temperature read by sensor in Celsius
         */
        float temperature();
};