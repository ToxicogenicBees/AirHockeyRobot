#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include "Types/Point2.hpp"

class Graph {
    private:#include <opencv2/opencv.hpp>
        Point2<std::vector<double>> _data;

        Point2<size_t> _size = (100, 100);
        size_t _margin = 5;

        cv::Mat _window;

    public:
        Graph();

        void resize(Point2<size_t> size, size_t margin);
        void resize(Point2<size_t> size);
        void setMargin(size_t margin);

        void plot(const std::vector<double>& x, const std::vector<double>& y);
        void plot(const std::vector<Point2<double>>& points);

        Point2<size_t> size() { return _size; }
        size_t margin() { return _margin; }
};