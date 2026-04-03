#include "Motion/MovingObject.hpp"

MovingObject::MovingObject(double radius)
    :_RADIUS(radius) {}

void MovingObject::moveTo(const Point2<double>& new_pos, double dt) {
    // Get access to lock
    std::lock_guard<std::mutex> lock(_access_locational_data);
    
    // Update velocity and position
    _orientation.direction = (new_pos - _orientation.position) / dt;
    _orientation.position = new_pos;

    // Reset timer
    _timer.reset();
}

void MovingObject::moveTo(const Point2<double>& new_pos) {
    moveTo(new_pos, 1e-6 * _timer.delta<std::chrono::microseconds>());
}

void MovingObject::orient(const Ray2<double>& orientation) {
    // Get access to lock
    std::lock_guard<std::mutex> lock(_access_locational_data);
    
    _orientation = orientation;
}

Ray2<double> MovingObject::orientation() const {
    // Get access to lock
    std::lock_guard<std::mutex> lock(_access_locational_data);

    // Return position
    return _orientation;
}

Ray2<double> MovingObject::futureOrientation(double dt) const {
    // Get position and velocity once to avoid blocking
    auto pos = position();
    auto vel = velocity();

    // Exit early if not moving
    if (vel.squaredMagnitude() <= Constants::FP_ERR)
        return {pos};

    // Raw displacement
    auto new_pos = pos + dt * vel;

    // Effective puck bounds (accounting for the radius of the puck)
    const Point2<double> MIN = _RADIUS * Point2<double>::one();
    const Point2<double> MAX = Constants::Table::SIZE - MIN;

    // Reflect off an axis
    auto reflect = [](double& new_pos, double& vel, double min, double max) {
        double range = max - min;
        double period = 2 * range;
        double mod = std::fmod(new_pos - min, period);

        if (mod < 0)
            mod += period;
        
        if (mod < range)
            new_pos = min + mod;
        else {
            new_pos = max - (mod - range);
            vel *= -1;
        }
    };

    // Reflect off each axis
    reflect(new_pos.x, vel.x, MIN.x, MAX.x);
    reflect(new_pos.y, vel.y, MIN.y, MAX.y);

    return { new_pos , vel };
}

std::vector<MovingObject::Timestamp> MovingObject::trajectory(bool include_return) const {
    const auto pos = position();
    const auto vel = velocity();

    // If the puck isn't moving, return early
    if (vel.magnitude() < 1e-8)
        return {};

    // If the puck is moving away and it should be ignored, return early
    if (vel.y > 0 && !include_return)
        return {};

    // Determine how long the puck will move for
    double effective_size = Constants::Table::SIZE.y - 2 * Constants::Puck::RADIUS;
    double effective_pos = pos.y - Constants::Puck::RADIUS;

    double time_of_arrival = (vel.y > 0 ? 2 * effective_size - effective_pos : -effective_pos) / vel.y;

    // Determine number of sample points
    const size_t num_samples = std::min(
        (size_t) (time_of_arrival / (1e-6 * Constants::SAMPLE_PERIOD)),
        Constants::MAX_SAMPLE_POINTS
    );
    
    double time_step = time_of_arrival / num_samples;
    
    // Calculate trajectory
    std::vector<Timestamp> trajectories;
    for (size_t i = 0; i <= num_samples; i++) {
        const double time = i * time_step;
        const auto orientation = futureOrientation(time);
        trajectories.push_back({time, orientation});
    }
    
    // Return trajectory
    return trajectories;
}

Point2<double> MovingObject::position() const {
    // Get access to lock
    std::lock_guard<std::mutex> lock(_access_locational_data);

    // Return position
    return _orientation.position;
}

Point2<double> MovingObject::velocity() const {
    // Get access to lock
    std::lock_guard<std::mutex> lock(_access_locational_data);

    // Return velocity
    return _orientation.direction;
}

double MovingObject::radius() const {
    return _RADIUS;
}
