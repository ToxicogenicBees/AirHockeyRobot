#pragma once

#include "Sensors/Sensor.hpp"
#include "Types/PinDef.hpp"

#include <stdint.h>

class TemperatureSensor : public Sensor {
    private:
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
