#pragma once

#include <opencv2/opencv.hpp>

class ColorFilter {
    private:
        // Color filtering ranges
        const cv::Scalar _MIN;
        const cv::Scalar _MAX;

        // Processing frames
        cv::Mat _threshold;
        cv::Mat _filtered;
        cv::Mat _hsv;

    public:
        /**
         * @brief Create a new color filter
         * 
         * @param min       Minimum color threshold (in HSV)
         * @param max       Maximum color threshold (in HSV)
         */
        ColorFilter(const cv::Scalar& min, const cv::Scalar& max)
            : _MIN(min), _MAX(max) {}

        /**
         * @brief Filter a frame using this filter's range
         * 
         * @param frame The frame to be filtered
         */
        void filter(const cv::Mat& frame) {
            // Convert inverted captured image from BGR to HSV colorspace
            cv::cvtColor(frame, _hsv, cv::COLOR_BGR2HSV);

            // Filter image to find the specific color
            cv::inRange(_hsv, _MIN, _MAX, _threshold);

            // Filter image to show only specific colors from orignal image
            _filtered = cv::Mat();
            cv::bitwise_and(frame, frame, _filtered, _threshold);
        }

        /**
         * @brief Get the threshold frame from the last filtered frame
         * 
         * @return Threshold frame of the last filter
         */
        cv::Mat threshold() const {
            return _threshold;
        }

        /**
         * @brief Get the threshold frame from the last filtered frame
         * 
         * @return Threshold frame of the last filter
         */
        cv::Mat filtered() const {
            return _filtered;
        }

        /**
         * @brief Get the threshold frame from the last filtered frame
         * 
         * @return Threshold frame of the last filter
         */
        cv::Mat hsv() const {
            return _hsv;
        }
};