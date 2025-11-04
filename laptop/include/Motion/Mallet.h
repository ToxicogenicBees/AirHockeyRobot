#pragma once

#include "Motion/MovingObject.h"
#include "Types/Matrix.hpp"
#include "Types/Point2.hpp"
#include "Types/Point3.hpp"
#include "Constants.h"

class Mallet {
    private:
        static constexpr double _TARGET_ERR         // Allowed tolerance before changing target positions
            = 0.05 * Constants::Mallet::RADIUS;  

        static Point2<double> _prev_target;         // Previously chosen target position
        static MovingObject _mallet;                // Moving object used for the mallet

    public:
        /***
         * @brief Calculates how long it would take the mallet to reach this point from where it currently is
         * 
         * @param pos   The position the mallet is trying to move to
         * 
         * @return How long it takes the mallet to reach this point, in seconds
         */
        static double timeToReach(const Point2<double>& pos);

        /***
         * @brief Determines if the mallet can reach this point
         * 
         * @param timestamp   The (x-position, y-position) to check
         * 
         * @return If the point is within range for the mallet
         */
        static bool canReach(const Point2<double> pos);

        /**
         * @brief Determines the target location for the mallet to go to
         * 
         * @param timestamps    The (x-position, y-position, time-of-arrival) timestamps to check
         * 
         * @return The point the mallet is targetting
         */
        static Point2<double> chooseTarget(const Matrix<Point3<double>>& timestamps);

        /**
         * @brief Updates the internal state of the object, traveling to the new position over the given time range
         * 
         * @param new_pos   The position the object is at now
         * @param msec      The time difference in microseconds between position updates,
         *                  defaults to the default sample time
         */
        static void moveTo(const Point2<double>& new_pos, int64_t micsec = -1);

        /**
         * @brief Sets the position and velocity of the object
         * 
         * @param pos   The desired velocity of the object, defaults to (0, 0)
         * @param vel   The desired velocity of the object, defaults to (0, 0)
         */
        static void orient(const Point2<double>& pos = {0.0, 0.0}, const Point2<double>& vel = {0.0, 0.0});

        /**
         * @brief Returns the current position of the object, in inches
         * 
         * @return The position of the object, in inches
         */
        static Point2<double> position();

        /**
         * @brief Returns the current velocity of the object, in inches/sec
         * 
         * @return The velocity of the object, in inches/sec
         */
        static Point2<double> velocity();
};