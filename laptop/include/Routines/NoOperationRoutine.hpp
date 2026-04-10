#pragma once

#include "Routines/Routine.hpp"

class NoOperationRoutine : public Routine {
    public:
        /**
         * @brief Create a new routine
         */
        NoOperationRoutine() = default;

        /**
         * @brief Destroy the routine
         */
        virtual ~NoOperationRoutine() = default;

        /**
         * @brief Calculates an appropriate mallet action for this routine
         */
        void updateTarget() override;
};