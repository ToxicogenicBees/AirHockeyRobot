#pragma once

#include <optional>

#include "Types/VelocityProfile.hpp"
#include "Motion/MovingObject.hpp"
#include "Types/Point2.hpp"
#include "Types/Ray2.hpp"

class Routine {
    protected:
        using Target = std::pair<Point2<double>, VelocityProfile>;

        // Reference to the table's mallet
        inline static MovingObject* _mallet = nullptr;

        // Previous transmission data
        inline static Target _prev_target = {Constants::Mallet::HOME, {0, 0, 0, 0}};

        /**
         * @brief Gets the time it takes the mallet to reach a position
         * 
         * @return The time it takes the mallet to reach this position
         */
        static double _timeToReach(const Point2<double>& position);

        /**
         * @brief Checks if the mallet can reach this position
         * 
         * @return If the mallet can reach this position
         * @returns -1 if the mallet cannot reach
         */
        static bool _canReach(const Point2<double>& position);

        /**
         * @brief Sets up the target and velocity profile to go to the home position
         */
        static void _travelHome();

    public:
        /**
         * @brief Create a new routine
         */
        Routine() = default;

        /**
         * @brief Pass a mallet pointer to all routines
         * 
         * @param mallet The mallet pointer
         */
        static void setMallet(MovingObject* mallet);

        /**
         * @brief Destroy the routine
         */
        virtual ~Routine() = default;

        /**
         * @brief Attempt to transmit the target, ignoring if the target is similar to the previous
         * 
         * @param target    The desired target
         */
        static void softTransmit(const Target& target);

        /**
         * @brief Attempt to transmit the target, ignoring if the target is similar to the previous
         * 
         * @param position  The target position
         */
        static void softTransmit(const Point2<double>& position);

        /**
         * @brief Attempt to transmit the target, ignoring if the target is similar to the previous
         * 
         * @param velocity  The target velocity
         */
        static void softTransmit(const VelocityProfile& velocity);

        /**
         * @brief Transmit the desired target
         *          
         * @param target  The desired target
         */
        static void transmit(const Target& target);

        /**
         * @brief Transmit the desired target
         *
         * @param position  The target position
         */
        static void transmit(const Point2<double>& position);

        /**
         * @brief Transmit the desired target
         *
         * @param velocity  The target velocity
         */
        static void transmit(const VelocityProfile& velocity);

        /**
         * @brief Get the current target position
         * 
         * @return The current target position
         */
        static Target target();

        /**
         * @brief Calculates an appropriate mallet action for this routine
         */
        virtual void updateTarget() = 0;
};
