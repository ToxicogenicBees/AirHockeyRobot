#include "Motion/MovingObject.h"

void MovingObject::readPosition(const Point2<double>& new_pos, double timestep) {
    _vel = (new_pos - _pos) / timestep;
    _pos = new_pos;
}

void MovingObject::init(const Point2<double>& pos) {
    _pos = pos;
    _initialized = true;
}