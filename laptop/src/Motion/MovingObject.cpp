#include "Motion/MovingObject.h"

void MovingObject::moveTo(const Point2<double>& new_pos, int64_t micsec) {
    // Get access to lock
    std::lock_guard<std::mutex> lock(_access_locational_data);

    // Get time difference
    int64_t d_micsec = _timer.delta<std::chrono::microseconds>();
    _timer.reset();
    
    // Update velocity and position
    _vel = (new_pos - _pos) / (1e-6 * (micsec < 0 ? d_micsec : micsec));
    _pos = new_pos;
}

void MovingObject::orient(const Point2<double>& pos, const Point2<double>& vel) {
    // Get access to lock
    std::lock_guard<std::mutex> lock(_access_locational_data);
    
    _pos = pos;
    _vel = vel;
}

Point2<double> MovingObject::position() {
    // Get access to lock
    std::lock_guard<std::mutex> lock(_access_locational_data);

    // Return position
    return _pos;
}

Point2<double> MovingObject::velocity() {
    // Get access to lock
    std::lock_guard<std::mutex> lock(_access_locational_data);

    // Return velocity
    return _vel;
}