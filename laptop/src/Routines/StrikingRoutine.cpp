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
    // Gantry axes
    const double INV_SQRT_2 = 1.0 / sqrt(2.0);
    const Point2<double> A_AXIS = {INV_SQRT_2, INV_SQRT_2};
    const Point2<double> B_AXIS = {-INV_SQRT_2, INV_SQRT_2};

    // Strike through setup
    constexpr double STRIKE_THROUGH_INCHES = 3.0;
    constexpr double STRIKE_POINT_OFFSET = -(Constants::Mallet::RADIUS + Constants::Puck::RADIUS);

    // Position deviation
    constexpr double POSITION_DEVIATION_WEIGHT = 1.0;
    constexpr double POSITION_OFFSET_MAX = 8.0;

    // Velocity deviation
    constexpr double VELOCITY_DEVIATION_WEIGHT = 1.0;
    constexpr double VELOCITY_OFFSET_MAX = 3.14 / 16;

    // Max deviation
    constexpr double MAX_DEVIATION = 8.0;
}

std::optional<StrikePlan> StrikingRoutine::_createPlan(const Ray2<double>& orientation, const Ray2<double>& puck_target, double time) {
    // Invalid strike time
    if (time < 0) {
        return std::nullopt;
    }

    // Get setup point based on how long the gantry needs to
    // accelerate to the desired striking velocity

    // Linearly interpolate from min to max distance to accelerate to max speed
    auto [desired_strike_pos, strike_vel] = orientation;
    auto strike_speed = strike_vel.magnitude();
    double accel_dist = strike_speed / Constants::Mallet::MAX_SPEED_INCHES_PER_SECOND * (Constants::Mallet::INCHES_TO_ACCEL_TO_MAX_RPM - Constants::Mallet::MIN_ACCEL_INCHES) + Constants::Mallet::MIN_ACCEL_INCHES;

    // Determine the strike point
    auto true_strike_pos = desired_strike_pos + STRIKE_POINT_OFFSET * strike_vel.normal();
    auto strike_through_pos = true_strike_pos + STRIKE_THROUGH_INCHES * strike_vel.normal();

    // Determine setup point
    Point2<double> setup_point = true_strike_pos - accel_dist * strike_vel.normal();
    
    // Time it will take to get from setup_point to true_strike_pos
    // assuming acceleration over a distance (Kaden's method)
    double min_speed = Constants::Mallet::MIN_RPM / Constants::Mallet::MAX_RPM * Constants::Mallet::MAX_SPEED_INCHES_PER_SECOND;
    double time_to_strike = accel_dist / (strike_speed - min_speed) * log(1 + (strike_speed - min_speed) / min_speed);

    if (time_to_strike > time || time_to_strike < Constants::FP_ERR) {
        return std::nullopt;
    }

    // Check if either required point is out of the mallet's available range of motion
    auto out_of_bounds = [](const Point2<double> p) {
        return p.x < Constants::Mallet::LIMIT_BL.x
            || p.y < Constants::Mallet::LIMIT_BL.y
            || p.x > Constants::Mallet::LIMIT_TR.x
            || p.y > Constants::Mallet::LIMIT_TR.y;
    };
    if (out_of_bounds(setup_point) || out_of_bounds(true_strike_pos) || out_of_bounds(strike_through_pos)) {
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
    double rpm_scale_a = std::abs( setup_direction.scalarProjection(A_AXIS) );
    double rpm_scale_b = std::abs( setup_direction.scalarProjection(B_AXIS) );
    rpm_to_setup *= rpm_scale_a > rpm_scale_b ? rpm_scale_a : rpm_scale_b;

    // Required RPM would cause the stepper motors to slip without an acceleration curve
    if (rpm_to_setup > Constants::Mallet::MIN_RPM) {
        return std::nullopt;
    }

    // Return strike
    return StrikePlan(setup_point, time_to_setup, rpm_to_setup, {strike_through_pos, strike_vel}, time_to_strike, accel_dist);
}

double StrikingRoutine::_deviation(const StrikePlan& plan, const Ray2<double>& puck_target, double *time_update) {
    // Fetch trajectory and return if it's empty
    auto trajectory = Table::puck().trajectory();
    if (trajectory.empty())
        return std::numeric_limits<double>::max();

    // Get strike orientation components
    auto [strike_puck_pos, puck_vel_old] = puck_target;

    double second_least_deviation_time = 0;
    double least_deviation_time = 0;
    
    // Find minimum deviation
    double min_deviation = std::numeric_limits<double>::max();
    double distance_norm;
    double velocity_norm;

    for (auto [time, orientation] : trajectory) {
        // Get puck orientation components
        auto [puck_pos, puck_vel] = orientation;

        // Calculate deviation components
        auto distance = (puck_pos - strike_puck_pos).magnitude();
        auto velocity_angle = puck_vel.angle(puck_vel_old);

        // Normalize components
        distance_norm = distance / POSITION_OFFSET_MAX;
        velocity_norm = velocity_angle / VELOCITY_OFFSET_MAX;

        // Get deviation
        double deviation = POSITION_DEVIATION_WEIGHT * distance_norm
            + VELOCITY_DEVIATION_WEIGHT * velocity_norm;

        // Save if this deviation is smaller
        if (deviation < min_deviation) {
            min_deviation = deviation;

            second_least_deviation_time = least_deviation_time;
            least_deviation_time = time;
        }
    }

    // std::clog << min_deviation << " " << velocity_norm << " " << distance_norm << "\n\n";

    *time_update = (second_least_deviation_time + least_deviation_time)/2;
    
    // std::clog << min_deviation << "\n";
    return min_deviation;
}

bool StrikingRoutine::strike(const Ray2<double>& orientation,  const Ray2<double>& puck_target, double time) {
    // Create a strike plan
    auto plan = _createPlan(orientation, puck_target, time);

    // Plan was impossible to create
    if (!plan) {
        return false;
    }

    // Start moving to the setup point
    softTransmit({ 0, 0, plan->setupRPM(), plan->setupRPM() });
    softTransmit(plan->setupPoint());

    // Wait for the setup to complete, checking that the trajectory is unchanged in the meantime
    double time_to_puck_get_in_position = -1;

    while (plan->elapsedTime() < plan->setupTime()) {
        // Puck has deviated too far from the expected trajectory
        if (_deviation(*plan, puck_target, &time_to_puck_get_in_position) > MAX_DEVIATION) {
            // std::cout << "Too much deviation!!!\n\n";
            return false;
        }
    }

    if (time_to_puck_get_in_position - plan->strikeTime() > 0.001) {
        std::this_thread::sleep_for(std::chrono::microseconds((int64_t)(1e6 * (time_to_puck_get_in_position - plan->strikeTime()) )));
    } 
    
    // else if (time_to_puck_get_in_position - plan->strikeTime() < 0.001) {
    //     // abandon strike if puck actually moving faster than expected
    //     return false;
    // }

    // Start the strike motion
    double accel_percent = plan->accelerationDistance() / (plan->strikePoint() - plan->setupPoint()).magnitude();
    double rpm_at_strike = plan->strikeVelocity().magnitude() / Constants::Mallet::MAX_SPEED_INCHES_PER_SECOND * Constants::Mallet::MAX_RPM;
    
    // Scalar projection of position vector onto CoreXY movement axis
    // divide the scalar projection value over the total movement distance to
    // get percent motor speed of the max speed
    auto displacement = plan->strikePoint() - plan->setupPoint();
    auto rpm_scale_a = std::abs( displacement.scalarProjection(A_AXIS) ) / plan->accelerationDistance();
    auto rpm_scale_b = std::abs( displacement.scalarProjection(B_AXIS) ) / plan->accelerationDistance();
    rpm_at_strike *= rpm_scale_a > rpm_scale_b ? rpm_scale_a : rpm_scale_b;

    softTransmit({ accel_percent, 1-accel_percent, (uint16_t) Constants::Mallet::MIN_RPM, (uint16_t) rpm_at_strike});
    softTransmit(plan->strikePoint());

    // Wait for the strike motion to complete before returning control
    // Add some slight additional time to complete the movement + decellerate
    auto strike_time = plan->strikeTime() + 0.5;
    std::this_thread::sleep_for(std::chrono::microseconds((int64_t)(1e6 * strike_time)));

    return true;
}
