#ifndef DISTANCESENSOR_H
#define DISTANCESENSOR_H

#include "Sensors/Sensor.hpp"
#include "Types/PinDef.h"

#include <stdint.h>

class DistanceSensor : public Sensor {
    private:
        static double _speed_of_sound;

    public:
        /**
         * @brief Creates an ultrasonic distance sensor
         * 
         * @param trig  The trigger pin
         * @param echo  The echo pin
         */
        DistanceSensor(PinDef& trig, PinDef& echo);

        /**
         * @brief Calibrate the distance sensor based on temperature
         */
        static void calibrate(double temperature);

        /**
         * @brief Yields and returns the current distance measurement of the sensor
         * 
         * @return The distance read by the sensor, in millimeters
         */
        double distance();
};

#endif
