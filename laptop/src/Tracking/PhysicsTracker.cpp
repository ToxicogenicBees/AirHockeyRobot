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
            (int) (PIXELS_PER_INCH * (Constants::Table::SIZE.y - inches.y))
        };
    }
}

PhysicsTracker::PhysicsTracker()
    : _overlay(inchesToPixels),
      _render(cv::Mat::zeros(PIXELS_PER_INCH * Constants::Table::SIZE.y, PIXELS_PER_INCH * Constants::Table::SIZE.x, CV_8UC3))
{}

void PhysicsTracker::init() {
    // Seed RNG
    std::srand(std::time(0));

    // Fetch random puck velocity
    double angle = M_PI * (45 + std::rand() % 90) / 180; 
    Point2<double> velocity = {
        Constants::Puck::SPEED * std::cos(angle),
        Constants::Puck::SPEED * std::sin(angle)
    };

    // Orient the puck
    _puck->orient({Constants::Puck::HOME, velocity});

    // Overlay settings
    _overlay.malletTarget({{255, 255, 255}, 2});
    _overlay.puckTrajectory({{0, 0, 255}, 2});
    _overlay.puck({{0, 0, 255}, 2});
    _overlay.mallet({{255, 255, 255}, 2});
    _overlay.humanGoal({{255, 127, 0}, 3});
    _overlay.robotGoal({{0, 255, 0}, 3});
}

void PhysicsTracker::track() {
    // Time since last track
    double dt = 1e-6 * _timer.delta<std::chrono::microseconds>();
    _timer.reset();

    // Get positions and velocities
    auto [mallet_position, mallet_velocity] = Table::mallet().orientation();
    auto [puck_position, puck_velocity] = _puck->orientation();

    auto r = Constants::Mallet::RADIUS + Constants::Puck::RADIUS;
    auto p = puck_position - mallet_position;      // relative position
    auto v = puck_velocity - mallet_velocity;      // relative velocity

    auto apply_drag = [&](double time_step) {
        auto position = _puck->position();
        auto velocity = _puck->velocity();
        auto new_velocity = velocity * std::exp(-0.05 * time_step);
        _puck->orient({position, new_velocity});
    };

    double rel_speed_sq = v.squaredMagnitude();

    // Handle near-zero relative velocity (same direction / stuck)
    if (rel_speed_sq < Constants::FP_ERR) {
        // Either no collision or puck is sliding along mallet
        _puck->orient(_puck->futureOrientation(dt));
        apply_drag(dt);
        return;
    }

    // Solve quadratic for collision time
    auto a = rel_speed_sq;
    auto b = 2 * p.dot(v);
    auto c = p.squaredMagnitude() - (r * r);

    double t = std::numeric_limits<double>::infinity();

    if (std::abs(a) < Constants::FP_ERR) {
        // Degenerate case: treat as linear
        if (std::abs(b) > Constants::FP_ERR) {
            t = -c / b;
        }
    } else {
        double discriminant = b * b - 4 * a * c;
        if (discriminant >= 0) {
            double sqrt_disc = std::sqrt(discriminant);
            double t1 = (-b + sqrt_disc) / (2 * a);
            double t2 = (-b - sqrt_disc) / (2 * a);

            if (t1 >= 0 && t1 <= dt) t = t1;
            if (t2 >= 0 && t2 <= dt) t = std::min(t, t2);
        }
    }

    if (!std::isfinite(t) || t > dt) {
        // No collision within this frame
        _puck->orient(_puck->futureOrientation(dt));
        apply_drag(dt);
        return;
    }

    // Move puck to collision point
    _puck->orient(_puck->futureOrientation(t));
    apply_drag(t);
    puck_position = _puck->position();
    mallet_position += mallet_velocity * t;

    // Compute collision normal
    auto n_vec = puck_position - mallet_position;
    double n_len_sq = n_vec.squaredMagnitude();

    if (n_len_sq < Constants::FP_ERR) {
        // Overlapping centers, pick arbitrary normal
        n_vec = Point2<double>{0, 1};
    } else {
        n_vec = n_vec.normal(); // normalized collision normal
    }

    // Elastic collision update (along normal only)
    auto rel_vel = puck_velocity - mallet_velocity;
    double rel_dot = rel_vel.dot(n_vec);

    if (std::abs(rel_dot) > Constants::FP_ERR) {
        auto new_velocity = puck_velocity
            + (1 + Constants::Table::COEF_REST) * (-rel_dot) * n_vec;
        _puck->orient({puck_position + Constants::Puck::RADIUS * new_velocity.normal(), new_velocity});
    }

    // Apply remaining drag for the rest of the timestep
    apply_drag(dt - t);
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
