#pragma once

#include "Types/Point3.hpp"
#include "Types/Matrix.hpp"
#include "MovingObject.h"

class Puck : public MovingObject {
    public:
        Puck() = default;
        ~Puck() = default;

        Matrix<Point3<double>> estimateTrajectory(size_t num_points);
};