#ifndef TEMPERATURESENSOR_H
#define TEMPERATURESENSOR_H

#include "Sensors/Sensor.h"
#include "Types/PinDef.h"

#include <stdint.h>

class TemperatureSensor : public Sensor {
    private:
        // TMP6131LPGM thermistor
        // Transfer function for temperature vs. Vsense voltage
        // is given in the TI Thermistor design tool
        static constexpr double _THRM_A0 = -2.885698e2;
        static constexpr double _THRM_A1 = 1.556236e2;
        static constexpr double _THRM_A2 = 7.191258e1;
        static constexpr double _THRM_A3 = -5.134061e1;
        static constexpr double _THRM_A4 = 1.244030e1;

        // ADC convertion ratio
        static constexpr double _ADC_BIAS = 3.3f / 4095;

        // Sampling
        static constexpr double _TEMP_ERR = 5.0;
        static constexpr uint8_t _SAMPLES = 20;

        double _buffer[_SAMPLES];
        size_t _buf_ind = 0;
        double _prev_avg;

        double _sample();

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
        double temperature();

        /**
         * @brief Initialize the temperature sensor
         */
        void init() override;
};

#endif
