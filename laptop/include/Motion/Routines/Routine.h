#ifndef ROUTINE_H
#define ROUTINE_H

#include "Types/VelocityProfile.hpp"
#include "Types/Point2.hpp"

class Routine {
    protected:
        // The desired mallet velocity profile
        VelocityProfile _velocity_profile;

        // The desired target mallet position
        Point2<double> _target;

        /**
         * @brief Set a target velocity profile and position
         * 
         * @param velocity_profile  The desired mallet velocity profile
         * @param position          The desired mallet position
         */
        void _sendCommand(const VelocityProfile& velocity_profile, const Point2<double> position);

        /**
         * @brief Sets the mallet to move to its home position
         * 
         * @param velocity_profile  The desired mallet velocity profile
         */
        void _home(const VelocityProfile& velocity_profile);

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

    public:
        /**
         * @brief Create a new routine
         */
        Routine();

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
        virtual void transmitTarget();

        /**
         * @brief Get the current target position
         * 
         * @return The current target position
         */
        Point2<double> getTarget();
};

#endif
