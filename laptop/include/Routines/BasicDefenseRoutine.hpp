#pragma once

#include "Motion/MovingObject.hpp"
#include "Routines/Routine.hpp"

class BasicDefenseRoutine : public Routine {
    public:
        /**
         * @brief Create a new routine
         */
        BasicDefenseRoutine() = default;

        /**
         * @brief Calculates and transmits an appropriate mallet action for this routine
         */
        virtual void updateTarget() override;
};
