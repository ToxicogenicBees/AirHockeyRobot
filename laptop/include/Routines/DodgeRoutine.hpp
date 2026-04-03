#pragma once

#include "Motion/MovingObject.hpp"
#include "Routines/Routine.hpp"

class DodgeRoutine : public Routine {
    public:
        /**
         * @brief Create a new routine
         */
        DodgeRoutine() = default;

        /**
         * @brief Calculates and transmits an appropriate mallet action for this routine
         */
        virtual void updateTarget() override;
};
