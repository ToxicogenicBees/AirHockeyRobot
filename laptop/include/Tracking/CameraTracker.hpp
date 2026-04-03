#pragma once

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc.hpp"

#include "Tracking/PuckTracker.hpp"
#include "Motion/MovingObject.hpp"
#include "Types/ColorFilter.hpp"
#include "Visuals/Overlay.hpp"
#include "Types/Point2.hpp"
#include <utility>

class CameraTracker : public PuckTracker {
    private:
        // Color filter
        ColorFilter _filter;

        // OpenCV capturing
        cv::VideoCapture _capture;
        cv::Mat _frame;

        // Render overlay
        Overlay _overlay;

        // Previous capture info
        cv::Point _prev_pixels;

    public:
        /***
         * @brief   Create a new puck tracker
         * 
         * @param   puck        The puck being tracked
         * @param   min_color   The minimum color threshold (in HSV)
         * @param   max_color   The maximum color threshold (in HSV)
         */
        CameraTracker(MovingObject& puck, const cv::Scalar& min_color, const cv::Scalar& max_color);

        /**
         * @brief   Initialize the tracker
         */
        void init();

        /***
         * @brief   Captures a frame from the tracker and updates the tracked puck position
         */
        void track() override;

        /***
         * @brief   Displays the most recently captured frame on screen
         */
        void display() override;
};
