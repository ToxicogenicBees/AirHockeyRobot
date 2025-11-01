#pragma once

#include "Types/Point2.hpp"
#include "Constants.h"

class MovingObject {
    protected:
        Point2<double> _pos, _vel;
        bool _initialized = false;

    public:
        MovingObject() = default;
        virtual ~MovingObject() = default;

        void readPosition(const Point2<double>& new_pos, double timestep = Constants::SAMPLE_RATE);
        void init(const Point2<double>& pos);

        Point2<double> position() { return _pos; }
        Point2<double> velocity() { return _vel; }
        double initialized() { return _initialized; }
};