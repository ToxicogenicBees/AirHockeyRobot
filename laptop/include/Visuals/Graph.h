#pragma once

#include <opencv2/opencv.hpp>

#include "Types/Matrix.hpp"
#include "Types/Point2.hpp"

class Graph {
    private:#include <opencv2/opencv.hpp>
        cv::Mat _window;

    public:
        Graph();

        void plot(const Matrix<Point2<double>>& x, const Matrix<Point2<double>>& y);
}