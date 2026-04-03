#pragma once

#include "Motion/MovingObject.hpp"
#include "Routines/Routine.hpp"

class BasicDefenseRoutine : public Routine {
    public:
        /**
         * @brief Create a new routine
         * 
         * @param mallet Reference to the mallet
         */
        BasicDefenseRoutine(MovingObject& mallet);

        /**
         * @brief Destroy the routine
         */
        virtual ~BasicDefenseRoutine() = default;

        /**
         * @brief Calculates and transmits an appropriate mallet action for this routine
         */
        virtual void updateTarget() override;
};
