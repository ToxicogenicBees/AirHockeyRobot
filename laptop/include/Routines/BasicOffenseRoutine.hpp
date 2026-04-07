#pragma once

#include "Routines/StrikingRoutine.hpp"

class BasicOffenseRoutine : public StrikingRoutine {
    public:
        /**
         * @brief Create a new routine
         */
        BasicOffenseRoutine() = default;

        /**
         * @brief Calculates and transmits an appropriate mallet action for this routine
         */
        void updateTarget() override;
};