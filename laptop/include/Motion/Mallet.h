#pragma once

#include "Motion/MovingObject.h"
#include "Types/Point2.hpp"
#include "Types/Point3.hpp"

class Mallet : public MovingObject {
    public:
        Mallet() = default;
        ~Mallet() = default;

        double timeToReach(const Point2<double>& pos);
        bool canReach(const Point3<double>& timestamp);
};