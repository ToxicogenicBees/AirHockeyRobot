#include <random>
#include <cmath>

#include "Tracking/PhysicsTracker.hpp"
#include "Motion/Table.hpp"
#include "Types/Point2.hpp"

namespace {
    constexpr int PIXELS_PER_INCH = 10;

    cv::Point inchesToPixels(const Point2<double>& inches) {
        return {
            (int) (PIXELS_PER_INCH * inches.x),
            (int) (PIXELS_PER_INCH * inches.y)
        };
    }
}

PhysicsTracker::PhysicsTracker(MovingObject& puck)
    : PuckTracker(puck), _overlay(inchesToPixels),
      _render(cv::Mat::zeros(PIXELS_PER_INCH * Constants::Table::SIZE.y, PIXELS_PER_INCH * Constants::Table::SIZE.x, CV_8UC3))
{}

void PhysicsTracker::init() {
    // Seed RNG
    std::srand(std::time(0));

    // Fetch random puck velocity
    double speed_percent = (1 + std::rand() % 5) / 10.0;
    double angle = std::rand() % 180 / M_PI; 
    Point2<double> velocity = {
        speed_percent * Constants::Puck::SPEED * std::cos(angle),
        speed_percent * Constants::Puck::SPEED * std::sin(angle)
    };

    // Orient the puck
    _puck.orient({Constants::Puck::HOME, velocity});

    // Overlay settings
    _overlay.malletTarget({{255, 255, 255}, 2});
    _overlay.puckTrajectory({{0, 0, 255}, 2});
    _overlay.puck({{0, 0, 255}, 2});
    _overlay.mallet({{255, 255, 255}, 2});
    _overlay.humanGoal({{255, 127, 0}, 3});
    _overlay.robotGoal({{0, 255, 0}, 3});
}

void PhysicsTracker::track() {
    // Get time since last track
    double dt = 1e-6 * _timer.delta<std::chrono::microseconds>();
    _timer.reset();

    // Check if the puck collided with the mallet
    auto [mallet_position, mallet_velocity] = Table::mallet().orientation();
    auto [puck_position, puck_velocity] = _puck.orientation();

    auto r = Constants::Mallet::RADIUS + Constants::Puck::RADIUS;
    auto p = puck_position - mallet_position;
    auto v = puck_velocity - mallet_velocity;

    // |p + vt|^2 = r^2
    auto a = v.squaredMagnitude();
    auto b = 2 * p.dot(v);
    auto c = p.squaredMagnitude() - (r * r);

    if (std::abs(a) < Constants::FP_ERR) {
        // Puck inside of mallet, simply move forward like normal
        _puck.orient(_puck.futureOrientation(dt));
        return;
    }

    // Calculate time of collision
    auto discriminant = b * b - 4 * a * c;
    if (discriminant < 0) {
        // No collision ever happens
        _puck.orient(_puck.futureOrientation(dt));
        return;
    }

    auto t = (-b - std::sqrt(discriminant)) / (2 * a);
    if (t < 0 || t > dt) {
        // Collision in the past/after this step
        _puck.orient(_puck.futureOrientation(dt));
        return;
    }

    // Move to point of collision
    _puck.orient(_puck.futureOrientation(t));
    puck_position = _puck.position();
    mallet_position += mallet_velocity * t;

    // Update velocity
    auto n = (puck_position - mallet_position).normal();
    auto new_velocity = puck_velocity + (1 + Constants::Table::COEF_REST) * n.dot(mallet_velocity - puck_velocity) * n;
    _puck.orient({puck_position, new_velocity});

    // Move puck for the rest of the collision
    _puck.orient(_puck.futureOrientation(dt - t));
}

void PhysicsTracker::display() {
    // Clear render
    _render.setTo(cv::Scalar(0, 0, 0));

    // Overlay table data
    _overlay.overlay(_render);

    // Show render
    cv::imshow("Simulated Puck", _render);
    cv::waitKey(1);
}
