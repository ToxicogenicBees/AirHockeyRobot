#ifndef MOTIONTESTROUTINE_H
#define MOTIONTESTROUTINE_H

#include "Motion/Routines/Routine.h"
#include "Types/Timer.hpp"

class MotionTestRoutine : public Routine {
    private:
        Timer _timer;

    public:
        /**
         * @brief Create a new routine
         */
        MotionTestRoutine();

        /**
         * @brief Destroy the routine
         */
        virtual ~MotionTestRoutine() = default;

        /**
         * @brief Clones the routine
         * 
         * @returns A clone of the routine
         */
        virtual std::unique_ptr<Routine> clone() const override;

        /**
         * @brief Calculates and transmits an appropriate mallet action for this routine
         */
        virtual void updateTarget() override;
};

#endif
