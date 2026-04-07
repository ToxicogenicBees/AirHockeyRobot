#pragma once

#include "Types/Point2.hpp"
#include "Types/Timer.hpp"
#include "Types/Ray2.hpp"

class StrikePlan {
    private:
        const Point2<double> _SETUP_POINT;
        const uint16_t _SETUP_RPM;
        const double _SETUP_TIME;
    
        const Ray2<double> _STRIKE_ORIENTATION;
        const double _ACCELERATION_DISTANCE;
        const double _STRIKE_TIME;
        
        Timer _timer;

    public:
        /**
         * @brief Create a new strike plan
         * 
         * @param setup_point           The point to set up at
         * @param setup_time            The total setup time
         * @param setup_rpm             The RPM required for the setup point
         * @param strike_orientation    The strike orientation (position + velocity)
         * @param strike_time           The total strike time
         * @param accel_dist            The acceleration distance for the strike
         */
        StrikePlan(const Point2<double>& setup_point, double setup_time, uint16_t setup_rpm,
        const Ray2<double>& strike_orientation,double strike_time, double accel_dist)
            : _SETUP_POINT(setup_point),_SETUP_RPM(setup_rpm), _SETUP_TIME(setup_time),
              _STRIKE_ORIENTATION(strike_orientation), _ACCELERATION_DISTANCE(accel_dist),
              _STRIKE_TIME(strike_time) {}

        /**
         * @brief Get the setup point
         * 
         * @return The setup point
         */
        Point2<double> setupPoint() const {
            return _SETUP_POINT;
        }

        /**
         * @brief Get the setup point
         * 
         * @return The setup point
         */
        double setupTime() const {
            return _SETUP_TIME;
        }

        /**
         * @brief Get the setup RPM
         * 
         * @return The setup RPM
         */
        uint16_t setupRPM() const {
            return _SETUP_RPM;
        }

        /**
         * @brief Get the strike orientation
         * 
         * @return The strike orientation
         */
        Ray2<double> strikeOrientation() const {
            return _STRIKE_ORIENTATION;
        }

        /**
         * @brief Get the strike point
         * 
         * @return The strike point
         */
        Point2<double> strikePoint() const {
            return _STRIKE_ORIENTATION.position;
        }

        /**
         * @brief Get the strike velocity
         * 
         * @return The strike velocity
         */
        Point2<double> strikeVelocity() const {
            return _STRIKE_ORIENTATION.direction;
        }

        /**
         * @brief Get the strike time
         * 
         * @return The strike time
         */
        double strikeTime() const {
            return _STRIKE_TIME;
        }

        double accelerationDistance() const {
            return _ACCELERATION_DISTANCE;
        }

        /**
         * @brief Get the elapsed time, in seconds
         */
        double elapsedTime() const {
            return 1e-6 * _timer.delta<std::chrono::microseconds>();
        }
};
