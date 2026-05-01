#include "Tracking/PuckTracker.hpp"

PuckTracker::PuckTracker(TrackerOverlay::Converter converter)
    : _overlay(new TrackerOverlay(converter)) {}

PuckTracker::~PuckTracker() {
    if (_overlay)
        delete _overlay;
}

void PuckTracker::setPuck(MovingObject* puck) {
    _puck = puck;
}

void PuckTracker::init() {
    // Do nothing by default
}

void PuckTracker::display() {
    // Do nothing by default
}
