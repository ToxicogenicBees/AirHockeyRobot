#include <optional>
#include <thread>
#include <limits>

#include "Routines/StrikingRoutine.hpp"
#include "Types/StrikePlan.hpp"
#include "Comms/SerialLink.hpp"
#include "Comms/Packet.hpp"
#include "Motion/Table.hpp"
#include "Constants.hpp"

namespace {
    const double MAX_TRIANGLE_AREA = 4.0;
    const double INV_SQRT_2 = 1.0 / sqrt(2.0);
    const Point2<double> A_AXIS = {INV_SQRT_2, INV_SQRT_2};
    const Point2<double> B_AXIS = {-INV_SQRT_2, INV_SQRT_2};

    // Calculate the area formed by successive points on the puck's trajectory and
    // the provided point, returning the minimum area calculated
    double minTriangularArea(const Point2<double>& position) {
        // Area of the triangle where the position is the third vertex
        auto area_of_triangle = [position](const Point2<double>& p1, const Point2<double>& p2) {
            return 0.5 * fabs(
                p1.x * (p2.y - position.y)
                + p2.x * (position.y - p1.y)
                + position.x * (p1.y - p2.y) 
            );
        };

        double min_area = std::numeric_limits<double>::max();
        std::optional<Point2<double>> prev_pos;

        for (auto [time, orientation] : Table::puck().trajectory()) {
            // Initialize the first position
            if (!prev_pos) {
                prev_pos = orientation.position;
                continue;
            }

            // Find area of triangle
            auto area = area_of_triangle(*prev_pos, orientation.position);
            if (area < min_area)
                min_area = area;
        }
        
        return min_area;
    }
}

std::optional<StrikePlan> StrikingRoutine::_createPlan(const Ray2<double>& orientation, double time) {
    // Invalid strike time
    if (time < 0) {
        return std::nullopt;
    }

    // Get setup point based on how long the gantry needs to
    // accelerate to the desired striking velocity

    // Linearly interpolate from min to max distance to accelerate to max speed
    auto pos = orientation.position;
    auto vel = orientation.direction;
    auto speed = vel.magnitude();
    double accel_dist = speed / Constants::Mallet::MAX_SPEED_INCHES_PER_SECOND * (Constants::Mallet::INCHES_TO_ACCEL_TO_MAX_RPM - Constants::Mallet::MIN_ACCEL_INCHES) + Constants::Mallet::MIN_ACCEL_INCHES;

    Point2<double> setup_point = pos - accel_dist * vel.normal();

    // Time it will take to get from setup_point to pos assuming constant acceleration
    // assuming acceleration over a distance (Kaden's method)
    double min_speed = Constants::Mallet::MIN_RPM / Constants::Mallet::MAX_RPM * Constants::Mallet::MAX_SPEED_INCHES_PER_SECOND;
    double time_to_strike = accel_dist / (speed - min_speed) * log(1 + (speed - min_speed) / min_speed);

    if (time_to_strike > time || time_to_strike < Constants::FP_ERR) {
        return std::nullopt;
    }

    // Finally set strike point to an inch past the input pos so can decel after hitting it
    // go through pos
    auto strike_point = orientation.unit().endPoint();

    // Check if either required point is out of the mallet's available range of motion
    auto out_of_bounds = [](const Point2<double> p) {
        return p.x < Constants::Mallet::LIMIT_BL.x
            || p.y < Constants::Mallet::LIMIT_BL.y
            || p.x > Constants::Mallet::LIMIT_TR.x
            || p.y > Constants::Mallet::LIMIT_TR.y;
    };
    if (out_of_bounds(setup_point) || out_of_bounds(strike_point)) {
        return std::nullopt;
    }

    // Find time left to be able to go to setup point
    double time_to_setup = time - time_to_strike;

    // Not enouch setup time
    if (time_to_setup < Constants::FP_ERR) {
        return std::nullopt;
    }

    // Find distance to go to setup point
    auto mallet_position = _mallet->position();
    auto setup_displacement = setup_point - mallet_position;
    auto setup_direction = setup_displacement.normal();

    // Find required speed of motor with more steps to setup point
    double speed_to_setup = setup_displacement.magnitude() / time_to_setup;
    double rpm_to_setup = speed_to_setup / Constants::Mallet::MAX_SPEED_INCHES_PER_SECOND * Constants::Mallet::MAX_RPM;
    
    // Scalar projection of setup position normal onto CoreXY movement axis
    // to get percent motor speed of the max speed
    double rpm_scale_a = fabs( setup_direction.scalarProjection(A_AXIS) );
    double rpm_scale_b = fabs( setup_direction.scalarProjection(B_AXIS) );
    rpm_to_setup *= rpm_scale_a > rpm_scale_b ? rpm_scale_a : rpm_scale_b;

    // Required RPM would cause the stepper motors to slip without an acceleration curve
    if (rpm_to_setup > Constants::Mallet::MIN_RPM) {
        return std::nullopt;
    }

    // Return strike
    return StrikePlan(setup_point, time_to_setup, rpm_to_setup, {strike_point, orientation.direction}, time_to_strike, accel_dist);
}

bool StrikingRoutine::strike(const Ray2<double>& orientation, double time) {
    // Create a strike plan
    auto plan = _createPlan(orientation, time);

    // Plan was impossible to create
    if (!plan) {
        return false;
    }

    // Start moving to the setup point
    softTransmit({ 0, 0, plan->setupRPM(), plan->setupRPM() });
    softTransmit(plan->setupPoint());

    // Wait for the strike to complete, checking that the trajectory is unchanged in the meantime
    while (plan->elapsedTime() < plan->setupTime()) {
        // Puck has deviated too far from the expected trajectory
        if (minTriangularArea(plan->strikePoint()) > MAX_TRIANGLE_AREA)
            return false;
    }

    // Start the strike motion
    double accel_percent = plan->accelerationDistance() / (plan->strikePoint() - plan->setupPoint()).magnitude();
    double rpm_at_strike = plan->strikeVelocity().magnitude() / Constants::Mallet::MAX_SPEED_INCHES_PER_SECOND * Constants::Mallet::MAX_RPM;
    
    // Scalar projection of position vector onto CoreXY movement axis
    // divide the scalar projection value over the total movement distance to
    // get percent motor speed of the max speed
    auto displacement = plan->strikePoint() - plan->setupPoint();
    auto rpm_scale_a = fabs( displacement.scalarProjection(A_AXIS) ) / plan->accelerationDistance();
    auto rpm_scale_b = fabs( displacement.scalarProjection(B_AXIS) ) / plan->accelerationDistance();
    rpm_at_strike *= rpm_scale_a > rpm_scale_b ? rpm_scale_a : rpm_scale_b;

    softTransmit({ accel_percent, 0.05, (uint16_t) Constants::Mallet::MIN_RPM, (uint16_t) rpm_at_strike});
    softTransmit(plan->strikePoint());

    // Wait for the strike motion to complete before returning control
    std::this_thread::sleep_for(std::chrono::microseconds((int64_t)(1e6 * plan->strikeTime())));

    return true;
}
