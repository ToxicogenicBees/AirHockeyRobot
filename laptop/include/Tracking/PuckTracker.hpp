#pragma once

#include <opencv2/opencv.hpp>
#include <functional>

#include "Motion/MovingObject.hpp"

class PuckTracker {
    protected:
        // Puck being tracked
        MovingObject& _puck;

    public:
        /**
         * @brief Create a new PuckTracker
         * 
         * @param puck  Reference to the table's puck
         */
        PuckTracker(MovingObject& puck);

        /**
         * @brief Initialize any internal tracking logic
         */
        virtual void init();

        /**
         * @brief Locate and update the puck's position
         */
        virtual void track() = 0;

        /**
         * @brief Render the puck's position
         */
        virtual void display();
};