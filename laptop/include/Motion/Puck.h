#ifndef PUCK_H
#define PUCK_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "Motion/MovingObject.h"
#include "Types/Point3.hpp"
#include "Types/Ray2.hpp"

class Puck {
    private:
        static MovingObject _puck;  // Moving object used for the puck

    public:
        using Timestamp = std::pair<Ray2<double>, double>;

        /***
         * @brief Calculates a vector of trajectory (x-position, y-position, time-of-arrival) timestamps of the
         *        puck's path across the table from its current spot to the back wall, or an empty vector if it's moving
         *        away from the back wall
         * 
         * @param ignore_return Does not process if the puck is moving away from the robot
         * 
         * @return The vector of timestamps
         */
        static std::vector<Timestamp> estimateTrajectory(bool ignore_return = true);

        /***
         * @brief Returns the position and velocity of the puck in the future, if it were to continue traveling in a straight line
         *        from its current position and with it's current velocity, reguardless of where the mallet is
         * 
         * @return The reflected velocity
         */
        static Ray2<double> determineFutureOrientation(double dt);

        /***
         * @brief Returns the velocity the puck would have after intersecting with the mallet,
         *        assuming it's actively touching the mallet at it's current position
         * 
         * @return The reflected velocity
         */
        static Point2<double> reflectedVelocity();

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
         * @param orientation   The desired orientation of the object
         */
        static void orient(const Ray2<double>& orientation);

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
        static Point2<double> position();

        /**
         * @brief Returns the current velocity of the object, in inches/sec
         * 
         * @return The velocity of the object, in inches/sec
         */
        static Point2<double> velocity();
};

#endif
