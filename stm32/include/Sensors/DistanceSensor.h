#pragma once

#include "Sensors/Sensor.h"
#include "Types/PinDef.h"

#include <stdint.h>

class DistanceSensor : public Sensor {
    public:
        /**
         * @brief Creates an ultrasonic distance sensor
         * 
         * @param trig  The trigger pin
         * @param echo  The echo pin
         */
        DistanceSensor(PinDef& trig, PinDef& echo);

        /**
         * @brief Yields and returns the current distance measurement of the sensor
         * 
         * @return The distance read by the sensor, in millimeters
         */
        double distance();
};