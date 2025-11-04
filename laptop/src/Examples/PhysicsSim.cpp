#include <opencv2/opencv.hpp>
#include <algorithm>
#include <windows.h>
#include <chrono>
#include <thread>

#include "Motion/Mallet.h"
#include "Motion/Puck.h"
#include "Constants.h"

// Pixels per inch scalar
const double TIME_STEP = Constants::SAMPLE_PERIOD * 1e-6;
const uint8_t IMG_SCALE = 10;

// Canvas
cv::Mat canvas = cv::Mat::zeros(IMG_SCALE * Constants::Table::SIZE.y, IMG_SCALE * Constants::Table::SIZE.x, CV_8UC3);

// Processing step
void processingStep() {
    // Determine target mallet location
    Matrix<Point3<double>> timestamps = Puck::estimateTrajectory();

    Point2<double> target = Mallet::chooseTarget(timestamps);
    Point2<double> pos_error = target - Mallet::position();
    Point2<double> vel_error = -Mallet::velocity();

    const double Kp = 50.0;
    const double Kd = 8.0;

    Point2<double> control = Kp * pos_error + Kd * vel_error;

    // Clamp to max speed
    if (control.magnitude() > Constants::Mallet::SPEED)
        control = control.normal() * Constants::Mallet::SPEED;

    Mallet::orient(Mallet::position(), control);
}

// Physics step
void physicsStep() {
    // Step forward puck and mallet
    Point2<double> fut_mallet_pos = Mallet::position() + Mallet::velocity() * TIME_STEP;
    auto fut_puck_orientation = Puck::determineFutureOrientation(TIME_STEP);

    // If mallet and puck are colliding, determine when they collided and recover
    double expected_dist = Constants::Mallet::RADIUS + Constants::Puck::RADIUS;
    double cur_dist = (fut_mallet_pos - fut_puck_orientation.first).magnitude();
    double cur_time = TIME_STEP;

    // Objects will be colliding
    if (cur_dist <= expected_dist) {
        int iter = 0;
        double cur_time_low = 0.0;
        double cur_time_high = TIME_STEP;
        double mid_time = 0.0;

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
        fut_mallet_pos = Mallet::position() + Mallet::velocity() * TIME_STEP;
        fut_puck_orientation = Puck::determineFutureOrientation(TIME_STEP);

        Puck::orient(fut_puck_orientation.first, fut_puck_orientation.second);
        Mallet::moveTo(fut_mallet_pos, (TIME_STEP - cur_time) * 1e6);
    }

    // Objects won't be colliding
    else {
        Puck::orient(fut_puck_orientation.first, fut_puck_orientation.second);
        Mallet::moveTo(fut_mallet_pos, TIME_STEP * 1e6);
    }
}

// Render step
void renderStep() {
    // Clear image
    canvas.setTo(cv::Scalar(0, 0, 0));

    // Fetch puck/mallet locations
    Point2<double> mallet_pos = Mallet::position();
    Point2<double> puck_pos = Puck::position();

    // Draw puck
    cv::Point puck_center(IMG_SCALE * puck_pos.x, IMG_SCALE * (Constants::Table::SIZE.y - puck_pos.y));
    cv::circle(canvas, puck_center, IMG_SCALE * Constants::Puck::RADIUS, cv::Scalar(0, 0, 255), 2);

    // Draw puck trajectory
    auto ts = Puck::estimateTrajectory(false);
    for (auto t : ts) {
        cv::Point point_center(IMG_SCALE * t.x, IMG_SCALE * (Constants::Table::SIZE.y - t.y));
        cv::circle(canvas, point_center, 2, cv::Scalar(0, 0, 255), 2);
    }

    // Draw mallet
    cv::Point mallet_center(IMG_SCALE * mallet_pos.x, IMG_SCALE * (Constants::Table::SIZE.y - mallet_pos.y));
    cv::circle(canvas, mallet_center, IMG_SCALE * Constants::Mallet::RADIUS, cv::Scalar(255, 255, 255), 2);

    // Draw mallet trajectory
    Point2<double> t = Mallet::target();
    cv::Point point_center(IMG_SCALE * t.x, IMG_SCALE * (Constants::Table::SIZE.y - t.y));
    cv::circle(canvas, point_center, 3, cv::Scalar(255, 255, 255), 3);

    // Show updated image
    cv::imshow("PhysicsSim", canvas);
}

int main() {
    Puck::orient(Constants::Puck::HOME, Constants::Puck::SPEED * Point2<double>(0.5 * std::sqrt(2), 0.5 * std::sqrt(2)));
    Mallet::orient(Constants::Mallet::HOME);

    while (true) {
        // "Rendering" pipeline
        processingStep();
        physicsStep();
        renderStep();
        
        // Pause for a sample tick
        std::this_thread::sleep_for(std::chrono::microseconds(Constants::SAMPLE_PERIOD));

        // Break if requested
        if (cv::waitKey(10) == 27) break;
    }

    return 0;
}