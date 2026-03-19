#ifndef STRIKETESTROUTINE_H
#define STRIKETESTROUTINE_H

#include "Motion/Routines/BasicDefenseRoutine.h"
#include "Types/Timer.hpp"

class StrikeTestRoutine : public BasicDefenseRoutine {
    private:
        Timer _timer;

    public:
        /**
         * @brief Create a new routine
         */
        StrikeTestRoutine();

        /**
         * @brief Destroy the routine
         */
        virtual ~StrikeTestRoutine() = default;

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
