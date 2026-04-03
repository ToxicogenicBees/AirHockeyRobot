#pragma once

#include "Routines/Routine.hpp"
#include "Types/Timer.hpp"

class MotionTestRoutine : public Routine {
    private:
        Timer _timer;

    public:
        /**
         * @brief Create a new routine
         * 
         * @param mallet Reference to the mallet
         */
        MotionTestRoutine(MovingObject& mallet);

        /**
         * @brief Destroy the routine
         */
        virtual ~MotionTestRoutine() = default;

        /**
         * @brief Calculates and transmits an appropriate mallet action for this routine
         */
        virtual void updateTarget() override;
};
