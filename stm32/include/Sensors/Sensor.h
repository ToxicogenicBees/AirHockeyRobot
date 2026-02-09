#ifndef SENSOR_H
#define SENSOR_H

#include "Types/PinDef.h"

#include <stdint.h>

class Sensor {
    protected:
        const uint8_t _NUM_PINS;    // Number of pins in the array
        PinDef** _PINS;             // Array of pointers to the pins

    public:
        /***
         * @brief Create a new sensor
         * 
         * @param num_pins  The total number of pins related to the sensor
         * @param ...       Variadic list of pointers to PinDef objects
         */
        Sensor(uint8_t num_pins, ...);
        
        /**
         * @brief Destroy a sensor
         */
        virtual ~Sensor();

        /**
         * @brief Initialize a sensor and it's pins
         */
        virtual void init();
};

#endif
