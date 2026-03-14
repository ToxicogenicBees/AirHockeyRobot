#ifndef BASICDEFENSEROUTINE_H
#define BASICDEFENSEROUTINE_H

#include "Motion/Routines/Routine.h"

class BasicDefenseRoutine : public Routine {
    public:
        /**
         * @brief Create a new routine
         */
        BasicDefenseRoutine();

        /**
         * @brief Destroy the routine
         */
        virtual ~BasicDefenseRoutine() = default;

        /**
         * @brief Clones the routine
         * 
         * @returns A clone of the routine
         */
        std::unique_ptr<Routine> clone() const override;

        /**
         * @brief Calculates and transmits an appropriate mallet action for this routine
         */
        void updateTarget() override;
};

#endif
