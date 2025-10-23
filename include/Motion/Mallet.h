#pragma once

#include "../Types/Point3.hpp"
#include "../Types/Point2.hpp"
#include "../Constants.h"

class Mallet {
    private:
        const double _RADIUS = 2.0;     // Inches

        Point2<double> _cur_pos;

    public:
        Mallet() = default;

        void readPosition(const Point2<double>& new_pos);

        double timeToReach(const Point2<double>& pos);

        bool canReach(const Point3<double>& timestamp);
};