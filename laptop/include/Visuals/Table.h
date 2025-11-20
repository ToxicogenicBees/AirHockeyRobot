#pragma once

#include <opencv2/opencv.hpp>
#include <thread>

class Table {
    private:
        static cv::Mat _canvas;

        static bool _show_target_mallet;
        static bool _show_target_puck;

        static void _renderStep();
        static void _renderLoop();

    public:
        /**
         * @brief Renders the puck and mallet on the table, with the desired
         *        target settings in a detached thread
         * 
         * @return Detached thread running the rendering loop
         */
        static std::thread render();

        /**
         * @brief Sets whether the mallet target location should be rendered
         * 
         * @param state Whether the mallet target location should be rendered
         */
        static void setMalletTarget(bool state);

        /**
         * @brief Sets whether the puck target locations should be rendered
         * 
         * @param state Whether the puck target locations should be rendered
         */
        static void setPuckTargets(bool state);
};