#pragma once

#include "Tracking/PuckTracker.hpp"

class NoOperationTracker : public PuckTracker {
    public:
        /**
         * @brief Create a new PuckTracker
         */
        NoOperationTracker() = default;

        /**
         * @brief Locate and update the puck's position
         */
        void track();
};