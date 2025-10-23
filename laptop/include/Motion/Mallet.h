#pragma once

#include "../../../shared/include/Types/Point2.hpp"
#include "../../../shared/include/Types/Point3.hpp"
#include "MovingObject.h"

class Mallet : public MovingObject {
    public:
        Mallet() = default;
        ~Mallet() = default;

        double timeToReach(const Point2<double>& pos);
        bool canReach(const Point3<double>& timestamp);
};