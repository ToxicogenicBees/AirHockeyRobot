#pragma once

#include "Motion/MovingObject.hpp"
#include "Routines/Routine.hpp"
#include "Types/Timer.hpp"

class ManualRoutine : public Routine {
    private:
        Point2<double> _position = Constants::Mallet::HOME;
        Timer _timer;

    public:
        /**
         * @brief Create a new routine
         */
        ManualRoutine() = default;

        /**
         * @brief Calculates and transmits an appropriate mallet action for this routine
         */
        void updateTarget() override;
};
