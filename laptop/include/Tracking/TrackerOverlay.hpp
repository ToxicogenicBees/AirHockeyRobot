#pragma once

#include <opencv2/opencv.hpp>
#include <functional>
#include <optional>

#include "Types/Point2.hpp"

class TrackerOverlay {
    public:
        using Converter = std::function<cv::Point(const Point2<double>&)>;
        using Setting = std::pair<cv::Scalar, int>;

    private:
        // Inch to pixel converter
        Converter _converter;

        // Render options
        std::optional<Setting> _puck_trajectory;
        std::optional<Setting> _puck;
        std::optional<Setting> _mallet_target;
        std::optional<Setting> _mallet;
        std::optional<Setting> _robot_goal;
        std::optional<Setting> _human_goal;

    public:
        /**
         * @brief Create a new render
         * 
         * @param converter Conversion from inch position to pixel location
         */
        TrackerOverlay(Converter converter);

        /**
         * @brief Overlay the render settings onto the provided matrix
         * 
         * @param mat The matrix to be overlayed onto
         */
        void overlay(cv::Mat& mat);

        /**
         * @brief Set robot goal setting
         * 
         * @param setting Robot goal setting
         */
        void robotGoal(const Setting& setting);

        /**
         * @brief Set human goal setting
         * 
         * @param setting Human goal setting
         */
        void humanGoal(const Setting& setting);

        /**
         * @brief Set mallet target setting
         * 
         * @param setting Mallet target setting
         */
        void malletTarget(const Setting& setting);

        /**
         * @brief Set puck trajectory setting
         * 
         * @param setting Puck trajectory setting
         */
        void puckTrajectory(const Setting& setting);

        /**
         * @brief Set mallet setting
         * 
         * @param setting Mallet setting
         */
        void mallet(const Setting& setting);

        /**
         * @brief Set puck setting
         * 
         * @param setting Puck setting
         */
        void puck(const Setting& setting);
};