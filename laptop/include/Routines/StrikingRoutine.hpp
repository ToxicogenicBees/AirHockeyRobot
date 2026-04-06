#pragma once

#include "Routines/Routine.hpp"
#include "Types/Point2.hpp"
#include "Types/Ray2.hpp"

class StrikingRoutine : public Routine {
    public:
        // Strike result states
        enum class StrikeResult {
            STRIKE_IMPOSSIBLE,
            STRIKE_IN_PROGRESS,
            STRIKE_COMPLETE,
        };

        /**
         * @brief Create a new routine
         */
        StrikingRoutine() = default;

        /**
         * @brief Calculates the area of the triangles between successive puck trajectory points and
         *        the provided point, returning the smallest area
         * 
         * @param position The position to compare against
         * 
         * @return The area of the smallest triange
         * @retval -1 The puck is stationary/not on the table
         */
        double minAreaOffset(const Point2<double>& position) const;

        /**
         * @brief Strike at a given location, at a set speed, at a set time
         * 
         * @param orientation   The desired strike orientation (position + velocity)
         * @param time          The desired strike time
         * 
         * @return The result of the strike
         */
        StrikeResult strike(const Ray2<double>& orientation, double time);
};
