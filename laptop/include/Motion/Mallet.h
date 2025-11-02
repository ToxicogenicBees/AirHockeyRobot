#pragma once

#include "Motion/MovingObject.h"
#include "Types/Matrix.hpp"
#include "Types/Point2.hpp"
#include "Types/Point3.hpp"

class Mallet : public MovingObject {
    private:
        // Calculates the time to reach this point, squared
        double _squaredTimeToReach(const Point2<double>& pos);

    public:
        /***
         * @brief Calculates how long it would take the mallet to reach this point from where it currently is
         * 
         * @param pos   The position the mallet is trying to move to
         * 
         * @return How long it takes the mallet to reach this point, in seconds
         */
        double timeToReach(const Point2<double>& pos);

        /***
         * @brief Determines if the mallet can reach 
         * 
         * @param timestamp   The (x-position, y-position, time-of-arrival) timestamp to check
         * 
         * @return How long it takes the mallet to reach this point, in seconds
         */
        bool canReach(const Point3<double>& timestamp);

        /**
         * @brief Determines the target location for the mallet to go to
         * 
         * @param timestamps    The (x-position, y-position, time-of-arrival) timestamps to check
         * 
         * @return The point the mallet is targetting
         */
        Point2<double> chooseTarget(const Matrix<Point3<double>>& timestamps);
};