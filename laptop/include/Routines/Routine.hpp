#pragma once

#include <memory>

#include "Types/VelocityProfile.hpp"
#include "Motion/MovingObject.hpp"
#include "Types/Point2.hpp"
#include "Types/Ray2.hpp"

class Routine {
    protected:
        // Reference to the table's mallet
        MovingObject& _mallet;

        // The desired mallet velocity profile
        VelocityProfile _velocity_profile;

        // The desired target mallet position
        Point2<double> _target;

        // Strike result states
        enum class StrikeResult {
            STRIKE_IMPOSSIBLE,
            STRIKE_IN_PROGRESS,
            STRIKE_COMPLETE,
        };

        /**
         * @brief Gets the time it takes the mallet to reach a position
         * 
         * @return The time it takes the mallet to reach this position
         */
        double _timeToReach(const Point2<double>& position);

        /**
         * @brief Checks if the mallet can reach this position
         * 
         * @return If the mallet can reach this position
         * @returns -1 if the mallet cannot reach
         */
        bool _canReach(const Point2<double>& position);

        /**
         * @brief Strikes the mallet with a given orientation and time
         * 
         * @param orientation   The desired mallet orientation
         * @param time          The desired time
         */
        StrikeResult _strike(const Ray2<double>& orientation, double time);

        /**
         * @brief Sets up the target and velocity profile to go to the home position
         */
        void _targetHome();

    public:
        /**
         * @brief Create a new routine
         * 
         * @param mallet Reference to the mallet
         */
        Routine(MovingObject& mallet);

        /**
         * @brief Destroy the routine
         */
        virtual ~Routine() = default;

        /**
         * @brief Calculates an appropriate mallet action for this routine
         */
        virtual void updateTarget() = 0;

        /**
         * @brief Transmits the target mallet action over the SerialLink
         */
        void transmitTarget() const;

        /**
         * @brief Get the current target position
         * 
         * @return The current target position
         */
        Point2<double> target() const;
};
