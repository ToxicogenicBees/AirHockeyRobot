#pragma once

#include "Types/Matrix.hpp"
#include "Types/Point3.hpp"
#include "Motion/Puck.h"
#include "Constants.h"

class PuckTracking {
    private:
        // Puck
        static Puck _puck;

        // Control flow
        static bool _initialized;

    public:
        /***
         * @brief Initializes the tracking system as a whole
         */
        static void init();

        /***
         * @brief Tracks the location of the puck on the table, and runs any required internal states
         */
        static void locate();

        /**
         * @brief Updates the internal state of the object, traveling to the new position over the given time range
         * 
         * @param new_pos   The position the object is at now
         * @param msec      The time difference in microseconds between position updates,
         *                  defaults to the default sample time
         */
        static void moveTo(const Point2<double>& new_pos);

        /***
         * @brief Calculates a column vector of trajectory (x-position, y-position, time-of-arrival) timestamps of the
         *        puck's path across the table from its current spot to the back wall, or an empty vector if it's moving
         *        away from the back wall
         * 
         * @return The column vector of timestamps
         */
        static Matrix<Point3<double>> estimateTrajectory();

        /**
         * @brief Returns whether the tracker is initialized or not
         * 
         * @return Whether the tracker is initialized or not
         */
        static bool initialized() { return _initialized; }
};