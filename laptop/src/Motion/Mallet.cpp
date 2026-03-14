#include "Motion/Mallet.h"
#include "Constants.h"

#include <algorithm>
#include <cmath>

std::unique_ptr<Routine> Mallet::_routine = nullptr;
MovingObject Mallet::_mallet;

void Mallet::moveTo(const Point2<double>& new_pos, int64_t micsec) {
    _mallet.moveTo(new_pos, micsec);
}

void Mallet::orient(const Point2<double>& pos, const Point2<double>& vel) {
    _mallet.orient(pos, vel);
}

void Mallet::setRoutine(std::unique_ptr<Routine> routine) {
    _routine = std::move(routine);
}

void Mallet::updateTarget() {
    if (_routine)
        _routine->updateTarget();
}

void Mallet::transmitTarget() {
    if (_routine)
        _routine->transmitTarget();
}

Point2<double> Mallet::position() {
    return _mallet.position();
}

Point2<double> Mallet::velocity() {
    return _mallet.velocity();
}

Point2<double> Mallet::target() {
    if (_routine)
        return _routine->getTarget();
    return -Point2<double>::one();
}