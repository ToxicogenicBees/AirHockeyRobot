#ifndef PHYSICS_HPP
#define PHYSICS_HPP

#include <vector>

#include "Simulation/Table.h"
#include "Types/Point2.hpp"
#include "Types/Timer.hpp"
#include "Motion/Mallet.h"
#include "Motion/Puck.h"
#include "Constants.h"

#include <iostream>

class Physics {
    private:
        inline static Timer _timer;
        
        static void _processingStep();
        static void _physicsStep();

    public:
        static void step();
};

void Physics::_processingStep() {
    // Fetch mallet location data
    Point2<double> target = Mallet::target();
    Point2<double> pos_error = target - Mallet::position();
    Point2<double> vel_error = -Mallet::velocity();

    // Simple PD controller for simulation (replace later)
    const double K_P = 10.0;
    const double K_D = 0.8;

    Point2<double> control = K_P * pos_error + K_D * vel_error;

    // Clamp to max speed
    if (control.magnitude() > Constants::Mallet::SPEED)
        control = control.normal() * Constants::Mallet::SPEED;

    Mallet::orient(Mallet::position(), control);
}

void Physics::_physicsStep() {
    // Calculate time step since last update
    const double TIME_STEP = 1e-6 * _timer.delta<std::chrono::microseconds>();
    _timer.reset();

    // Step forward puck and mallet
    Point2<double> fut_mallet_pos = Mallet::position() + Mallet::velocity() * TIME_STEP;
    auto fut_puck_orientation = Puck::determineFutureOrientation(TIME_STEP);

    // If mallet and puck are colliding, determine when they collided and recover
    double expected_dist = Constants::Mallet::RADIUS + Constants::Puck::RADIUS;
    double cur_dist = (fut_mallet_pos - fut_puck_orientation.first).magnitude();
    double cur_time = TIME_STEP;

    // Objects will be colliding
    if (cur_dist <= expected_dist) {
        double cur_time_high = TIME_STEP;
        double cur_time_low = 0.0;
        double mid_time = 0.0;
        int iter = 0;

        // Binary search for collision
        while (iter++ < 50) {
            mid_time = 0.5 * (cur_time_low + cur_time_high);

            fut_mallet_pos = Mallet::position() + Mallet::velocity() * mid_time;
            fut_puck_orientation = Puck::determineFutureOrientation(mid_time);
            cur_dist = (fut_mallet_pos - fut_puck_orientation.first).magnitude();

            if (std::fabs(cur_dist - expected_dist) < 1e-8)
                break;

            if (cur_dist > expected_dist)
                cur_time_low = mid_time;
            else
                cur_time_high = mid_time;
        }

        cur_time = mid_time;

        // Step forward the first half of the collision
        Puck::orient(fut_puck_orientation.first, fut_puck_orientation.second);
        Mallet::moveTo(fut_mallet_pos, cur_time * 1e6);

        // Run collision calculation
        Point2<double> ref_vel = Puck::reflectedVelocity();
        Puck::orient(fut_puck_orientation.first, ref_vel);

        // Step forward the second half of the collision
        fut_mallet_pos = Mallet::position() + Mallet::velocity() * (TIME_STEP - cur_time);
        fut_puck_orientation = Puck::determineFutureOrientation(TIME_STEP - cur_time);

        Puck::orient(fut_puck_orientation.first, fut_puck_orientation.second);
        Mallet::moveTo(fut_mallet_pos, (TIME_STEP - cur_time) * 1e6);
    }

    // Objects won't be colliding
    else {
        Puck::orient(fut_puck_orientation.first, fut_puck_orientation.second);
        Mallet::moveTo(fut_mallet_pos, TIME_STEP * 1e6);
    }
}

void Physics::step() {
    Mallet::updateTarget();
    _processingStep();
    _physicsStep();
}

#endif
