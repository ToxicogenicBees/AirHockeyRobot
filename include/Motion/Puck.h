#pragma once

#include "../Types/Matrix.hpp"
#include "../Types/Point3.hpp"
#include "../Types/Point2.hpp"
#include "../Constants.h"

class Puck {
    private:
        const double _RADIUS = 1.25;    // Inches

        Point2<double> _prev_pos, _cur_pos, _velocity;

    public:
        Puck() = default;

        void readPosition(const Point2<double>& new_pos, double time_step);

        Matrix<Point3<double>> estimateTrajectory(size_t num_points);
};