#include "Motion/MovingObject.h"

int64_t MovingObject::_micsec() {
    // Get current time
    SampleTime now = Clock::now();

    // Find time difference between then and now
    int64_t msec = std::chrono::duration_cast<std::chrono::microseconds>(now - _prev_sample).count();
    _prev_sample = now;

    return msec;
}

void MovingObject::moveTo(const Point2<double>& new_pos, int64_t micsec) {
    // Get access to lock
    std::lock_guard<std::mutex> lock(_access_locational_data);

    // Get time difference
    int64_t d_micsec = _micsec();
    
    // Update velocity and position
    _vel = (new_pos - _pos) / (1e6 * (micsec < 0 ? d_micsec : micsec));
    _pos = new_pos;
}

void MovingObject::orient(const Point2<double>& pos, const Point2<double>& vel) {
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