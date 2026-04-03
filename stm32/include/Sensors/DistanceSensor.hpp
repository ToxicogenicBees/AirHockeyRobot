#pragma once

#include "Sensors/Sensor.hpp"
#include "Types/PinDef.hpp"

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

        /**
         * @brief Take multiple readings of the distance sensor and return the median.
         * 
         * @param n_samples  Number of samples to take
         * @return The median distance read by the sensor, in millimeters
         */
        double distanceBurstMedian(int n_samples);
};
