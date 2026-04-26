#pragma once

#include <vector>

#include "Types/PinDef.hpp"

class Sensor {
    protected:
        std::vector<PinDef*> _pins;

    public:
        /***
         * @brief Create a new sensor
         * 
         * @param args  Variadic list of pointers to PinDef objects
         */
        template <typename... Args>
        Sensor(Args&... args)
            : _pins{&args...} {}

        /***
         * @brief Destroys a sensor
         */
        virtual ~Sensor() = default;

        /**
         * @brief Initialize a sensor and it's pins
         */
        virtual void init() {
            for (auto* pin : _pins)
                pin->init();
        }
};
