#pragma once

#include "Routines/BasicDefenseRoutine.hpp"
#include "Types/Timer.hpp"

class StrikeTestRoutine : public BasicDefenseRoutine {
    private:
        Timer _timer;

    public:
        /**
         * @brief Create a new routine
         * 
         * @param mallet Reference to the mallet
         */
        StrikeTestRoutine(MovingObject& mallet);

        /**
         * @brief Destroy the routine
         */
        virtual ~StrikeTestRoutine() = default;

        /**
         * @brief Calculates and transmits an appropriate mallet action for this routine
         */
        virtual void updateTarget() override;
};
