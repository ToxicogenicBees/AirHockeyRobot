#pragma once

#include "Routines/Routine.hpp"
#include "Types/StrikePlan.hpp"
#include "Types/Point2.hpp"
#include "Types/Timer.hpp"
#include "Types/Ray2.hpp"

class StrikingRoutine : public Routine {
    private:
        /**
         * @brief Create a strike plan for the provided location, velocity, and time
         * 
         * @param orientation   The desired strike orientation (position + velocity)
         * @param time          The desired strike time
         * 
         * @return The created strike plan
         * @retval std::nullopt if the strike is impossible
         */
        std::optional<StrikePlan> _createPlan(const Ray2<double>& orientation, const Ray2<double>& puck_target, double time);

        /**
         * @brief Determine how far the desired plan deviates from the current puck trajectory
         * 
         * @param plan  The desired plan
         */
        double _deviation(const StrikePlan& plan, const Ray2<double>& puck_target, double *time_update);

    public:
        /**
         * @brief Create a new routine
         */
        StrikingRoutine() = default;

        /**
         * @brief Strike at a given location, at a set velocity, at a set time
         * 
         * @param orientation   The desired strike orientation (position + velocity)
         * @param puck_target   The predicted puck position and velocity at the passed time
         * @param time          The desired strike time
         * 
         * @return Whether or not the strike was successful
         */
        bool strike(const Ray2<double>& orientation, const Ray2<double>& puck_target, double time);
};
