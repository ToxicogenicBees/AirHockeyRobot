#ifndef TABLE_H
#define TABLE_H

#include <opencv2/opencv.hpp>
#include <thread>

class Table {
    private:
        static cv::Mat _canvas;

        static bool _show_relative_times;
        static bool _show_target_mallet;
        static bool _show_target_puck;

    public:
        /**
         * @brief Renders the puck and mallet on the table
         */
        static void render();

        /**
         * @brief Sets whether the relative motion times should be plotted
         * 
         * @param state Whether the relative motion times should be plotted
         */
        static void setRelativeTimes(bool state);

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

#endif
