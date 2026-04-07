#pragma once

#include "Routines/StrikingRoutine.hpp"
#include "Routines/Routine.hpp"
#include "Types/Timer.hpp"

class StrikeTestRoutine : public StrikingRoutine {
    public:
        /**
         * @brief Create a new routine
         */
        StrikeTestRoutine() = default;

        /**
         * @brief Calculates and transmits an appropriate mallet action for this routine
         */
        void updateTarget() override;
};
