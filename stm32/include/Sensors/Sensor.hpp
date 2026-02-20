#ifndef SENSOR_H
#define SENSOR_H

#include "Types/PinDef.h"

#include <array>

template <size_t N>
class Sensor {
    protected:
        std::array<PinDef*, N> _pins;

    public:
        /***
         * @brief Create a new sensor
         * 
         * @param num_pins  The total number of pins related to the sensor
         * @param ...       Variadic list of pointers to PinDef objects
         */
        Sensor(...);

        /**
         * @brief Initialize a sensor and it's pins
         */
        virtual void init();
};

template <size_t N>
Sensor<N>::Sensor(...) {
    va_list args;
    va_start(args, N);

    _pins.reserve(num_pins);
    for (size_t i = 0; i < N; ++i)
        _pins[i] = va_arg(args, PinDef*);

    va_end(args);
}

template <size_t N>
void Sensor<N>::init() {
    for (auto pin : _pins)
        pin->init();
}

#endif
