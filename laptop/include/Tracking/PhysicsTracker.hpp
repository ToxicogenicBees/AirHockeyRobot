#pragma once

#include "Tracking/PuckTracker.hpp"
#include "Tracking/TrackerOverlay.hpp"
#include "Types/Timer.hpp"

class PhysicsTracker : public PuckTracker {
    private:
        TrackerOverlay _overlay;
        cv::Mat _render;
        Timer _timer;

    public:
        /**
         * @brief Create a new PhysicsTracker
         */
        PhysicsTracker();

        /**
         * @brief Initialize any internal tracking logic
         */
        void init() override;

        /**
         * @brief Locate and update the puck's position
         */
        void track() override;

        /**
         * @brief Render the puck's position
         */
        void display() override;
};