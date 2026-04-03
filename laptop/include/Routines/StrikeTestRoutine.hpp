#pragma once

#include "Routines/Routine.hpp"
#include "Types/Timer.hpp"

class StrikeTestRoutine : public Routine {
    private:
        Timer _timer;

    public:
        /**
         * @brief Create a new routine
         */
        StrikeTestRoutine() = default;

        /**
         * @brief Calculates and transmits an appropriate mallet action for this routine
         */
        virtual void updateTarget() override;
};
