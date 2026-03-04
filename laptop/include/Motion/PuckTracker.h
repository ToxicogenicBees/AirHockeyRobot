#ifndef PUCKTRACKER_H
#define PUCKTRACKER_H

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc.hpp"

#include "Types/Point2.hpp"
#include <utility>

class PuckTracker {
    private:
        // Conversion ratio from pixels to inches
        double _INCHES_PER_PIXEL = 1.0 / 21.0;

        // Camera Ids
        int _DEVICE_ID;
        int _API_ID;

        // OpenCV frames
        cv::Mat _frame_threshold;
        cv::Mat _frame_filtered;
        cv::Mat _frame_hsv;
        cv::Mat _frame;

        // OpenCV video capture
        cv::VideoCapture _capture;

        // Color filtering ranges (defaults to Yellow)
        cv::Scalar _min;
        cv::Scalar _max;

        // Previous captures
        cv::Point _prev_pixels;

    public:
        /***
         * @brief   Create a new puck tracker
         */
        PuckTracker(int device_id, int api_id);

        void init();

        /***
         * @brief   Captures a frame from the tracker and updates the tracked puck position
         */
        void captureFrame();

        /***
         * @brief   Displays the most recently captured frame on screen
         */
        void displayFrame();

        /***
         * @brief   Sets the camera filtering range
         * 
         * @param   min   The minimum threshold
         * @param   max   The maximum threshold
         */
        void setFilteringRange(const cv::Scalar& min, const cv::Scalar& max);

        /***
         * @brief   Sets the camera filtering range to search for yellow objects
         */
        void filterForYellow();

        /***
         * @brief   Sets the camera filtering range to search for red objects
         */
        void filterForRed();

        /***
         * @brief   Sets the camera filtering range to search for green objects
         */
        void filterForGreen();

        /***
         * @brief   Gets the camera filtering range
         * 
         * @return  A pair of scalars, with the first being the min threshold and
         *          the second being the max threshold
         */
        std::pair<cv::Scalar, cv::Scalar> getFilteringRange();

        /***
         * @brief   Get the tracker's device id
         * 
         * @return  The device id
         */
        int getDeviceId();

        /***
         * @brief   Get the tracker's device id
         * 
         * @return  The api id
         */
        int getApiId();
};

#endif
