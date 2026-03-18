#ifndef MOVING_OBJECT_H
#define MOVING_OBJECT_H

#include "Types/Point2.hpp"
#include "Types/Timer.hpp"
#include "Types/Ray2.hpp"
#include "Constants.h"

#include <chrono>
#include <mutex>

class MovingObject {
    private:
        Ray2<double> _orientation;              // Position and velocity of the object
        std::mutex _access_locational_data;     // Mutex lock to guard access to position/velocity data
        Timer _timer;                           // Microsecond timer

    public:
        MovingObject() = default;
        ~MovingObject() = default;

        /**
         * @brief Updates the internal state of the object, traveling to the new position over the given time range
         * 
         * @param new_pos   The position the object is at now
         * @param msec      The time difference in microseconds between position updates,
         *                  defaults to the default sample time
         */
        void moveTo(const Point2<double>& new_pos, int64_t micsec = -1);

        /**
         * @brief Sets the position and velocity of the object
         * 
         * @param orientation   The desired orientation of the object
         */
        void orient(const Ray2<double>& orientation);

        /**
         * @brief Returns the current position of the object, in inches and inches/sec
         * 
         * @return The orientation of the object, in inches and inches/sec
         */
        Ray2<double> orientation();

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

#endif
