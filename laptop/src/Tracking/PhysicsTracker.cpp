#include <random>
#include <cmath>

#include "Tracking/PhysicsTracker.hpp"
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

    // Move puck forward in time
    auto orientation = _puck.futureOrientation(dt);
    _puck.orient(orientation);
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
