#pragma once

#include "Types/Point2.hpp"
#include "Constants.h"

#include <chrono>
#include <mutex>

class MovingObject {
    protected:
        using Clock = std::chrono::high_resolution_clock;
        using SampleTime = std::chrono::time_point<Clock>;

        // Position and velocity of the object
        Point2<double> _pos = {0.0, 0.0}, _vel = {0.0, 0.0};
        std::mutex _access_locational_data;

        // Last sample time for the moving object
        SampleTime _prev_sample = Clock::now();

        // Gets time difference since last call, in microseconds
        int64_t _micsec();

    public:
        virtual ~MovingObject() = default;

        /**
         * @brief Updates the internal state of the object, traveling to the new position over the given time range
         * 
         * @param new_pos   The position the object is at now
         * @param msec      The time difference in microseconds between position updates,
         *                  defaults to the default sample time
         */
        void moveTo(const Point2<double>& new_pos, int64_t micsec = Constants::SAMPLE_RATE);

        /**
         * @brief Sets the position and velocity of the object
         * 
         * @param pos   The desired velocity of the object, defaults to (0, 0)
         * @param vel   The desired velocity of the object, defaults to (0, 0)
         */
        void orient(const Point2<double>& pos = {0.0, 0.0}, const Point2<double>& vel = {0.0, 0.0});

        /**
         * @brief Returns the current position of the object, in inches
         * 
         * @return The position of the object, in inches
         */
        Point2<double> position();

        /**
         * @brief Returns the current velocity of the object, in inches/sec
         * 
         * @return The velocity of the object, in inches/sec
         */
        Point2<double> velocity();
};