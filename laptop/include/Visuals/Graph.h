#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include "Types/Point2.hpp"

class Graph {
    private:#include <opencv2/opencv.hpp>
        cv::Mat _window;

    public:
        Graph();

        void plot(const std::vector<Point2<double>>& x, const std::vector<Point2<double>>& y);
}