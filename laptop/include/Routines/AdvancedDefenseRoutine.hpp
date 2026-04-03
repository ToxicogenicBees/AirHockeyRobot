#pragma once

#include "Routines/BasicDefenseRoutine.hpp"
#include "Routines/DodgeRoutine.hpp"

class AdvancedDefenseRoutine : public Routine {
    private:
        BasicDefenseRoutine _defense;
        DodgeRoutine _dodge;

        bool _dodging = false;

    public:
        /**
         * @brief Create a new routine
         */
        AdvancedDefenseRoutine() = default;

        /**
         * @brief Calculates an appropriate mallet action for this routine
         */
        void updateTarget() override;
};