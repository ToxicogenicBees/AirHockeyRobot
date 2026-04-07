#pragma once

#include "Routines/Routine.hpp"
#include "Types/Timer.hpp"

class MotionTestRoutine : public Routine {
    private:
        Timer _timer;

    public:
        /**
         * @brief Create a new routine
         */
        MotionTestRoutine() = default;

        /**
         * @brief Calculates and transmits an appropriate mallet action for this routine
         */
        void updateTarget() override;
};
