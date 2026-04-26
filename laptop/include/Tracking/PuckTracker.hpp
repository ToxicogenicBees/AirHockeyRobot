#pragma once

#include <opencv2/opencv.hpp>
#include <functional>
#include <memory>

#include "Tracking/TrackerOverlay.hpp"
#include "Motion/MovingObject.hpp"

class PuckTracker {
    protected:
        // Puck being tracked
        inline static MovingObject* _puck = nullptr;

        // Tracker overlay
        TrackerOverlay* _overlay = nullptr;

    public:
        /**
         * @brief Create a new PuckTracker
         */
        PuckTracker() = default;

        /**
         * @brief Create a new PuckTracker with a tracker overlay
         * 
         * @param converter Inch to pixels converter
         */
        PuckTracker(TrackerOverlay::Converter converter);

        /**
         * @brief Destroy a PuckTracker
         */
        virtual ~PuckTracker();

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