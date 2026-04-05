#pragma once

#include <opencv2/opencv.hpp>
#include <functional>

#include "Motion/MovingObject.hpp"

class PuckTracker {
    protected:
        // Puck being tracked
        inline static MovingObject* _puck = nullptr;

    public:
        /**
         * @brief Create a new PuckTracker
         */
        PuckTracker() = default;

        /**
         * @brief Pass a mallet pointer to all routines
         * 
         * @param puck  The puck pointer
         */
        static void setPuck(MovingObject* puck);

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