#pragma once

#include "Types/Point3.hpp"
#include "Types/Matrix.hpp"
#include "MovingObject.h"

class Puck : public MovingObject {
    public:
        /***
         * @brief Calculates a column vector of trajectory (x-position, y-position, time-of-arrival) timestamps of the
         *        puck's path across the table from its current spot to the back wall, or an empty vector if it's moving
         *        away from the back wall
         * 
         * @return The column vector of timestamps
         */

        Matrix<Point3<double>> estimateTrajectory();
};