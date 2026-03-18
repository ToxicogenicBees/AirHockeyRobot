#include "Motion/MovingObject.h"

void MovingObject::moveTo(const Point2<double>& new_pos, int64_t micsec) {
    // Get access to lock
    std::lock_guard<std::mutex> lock(_access_locational_data);

    // Get time difference
    int64_t d_micsec = _timer.delta<std::chrono::microseconds>();
    _timer.reset();
    
    // Update velocity and position
    _orientation.setDirection((new_pos - _orientation.position()) / (1e-6 * (micsec < 0 ? d_micsec : micsec)));
    _orientation.setPosition(new_pos);
}

void MovingObject::orient(const Ray2<double>& orientation) {
    // Get access to lock
    std::lock_guard<std::mutex> lock(_access_locational_data);
    
    _orientation = orientation;
}

Ray2<double> MovingObject::orientation() {
    // Get access to lock
    std::lock_guard<std::mutex> lock(_access_locational_data);

    // Return position
    return _orientation;
}

Point2<double> MovingObject::position() {
    // Get access to lock
    std::lock_guard<std::mutex> lock(_access_locational_data);

    // Return position
    return _orientation.position();
}

Point2<double> MovingObject::velocity() {
    // Get access to lock
    std::lock_guard<std::mutex> lock(_access_locational_data);

    // Return velocity
    return _orientation.direction();
}