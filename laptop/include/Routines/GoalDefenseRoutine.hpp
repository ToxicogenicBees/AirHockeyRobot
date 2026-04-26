#pragma once

#include "Routines/Routine.hpp"

class GoalDefenseRoutine : public Routine {
    public:
        /**
         * @brief Create a new routine
         */
        GoalDefenseRoutine() = default;

        /**
         * @brief Calculates an appropriate mallet action for this routine
         */
        void updateTarget() override;
};