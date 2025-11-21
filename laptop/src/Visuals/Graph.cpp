#include <algorithm>
#include <stdexcept>
#include <iostream>

#include "Visuals/Graph.h"

const Point2<double> Graph::DYNAMIC_RANGE = {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};

cv::Point Graph::_normalize(const Point2<double>& point, const Range& range) {
    Point2<double> normal = (point - range.first) / (range.second - range.first);

    return {
        (int) (_margin + (_size.x - 2 * _margin) * normal.x),
        (int) (_margin + (_size.y - 2 * _margin) * normal.y)
    };
}

void Graph::_resizeWindow(Point2<size_t> size) {
    _image = cv::Mat(size.y, size.x, CV_8UC3);
    cv::resizeWindow(_NAME, size.x, size.y);
}

Graph::Range Graph::_dynamicRange() {
    bool dyn_x = (_range.first.x == DYNAMIC_RANGE.x) && (_range.second.x == DYNAMIC_RANGE.x);
    bool dyn_y = (_range.first.y == DYNAMIC_RANGE.y) && (_range.second.y == DYNAMIC_RANGE.y);

    if (!dyn_x && !dyn_y)
        return _range;

    Point2<double> min = _data[0].data[0];
    Point2<double> max = _data[0].data[0];

    for (const DataSet& d : _data) {
        for (const Point2<double>& p : d.data) {
            min = {
                dyn_x ? std::min(min.x, p.x) : _range.first.x,
                dyn_y ? std::min(min.y, p.y) : _range.first.y,
            };
            
            max = {
                dyn_x ? std::max(max.x, p.x) : _range.second.x,
                dyn_y ? std::max(max.y, p.y) : _range.second.y,
            };
        }
    }

    return {min, max};
}

void Graph::_drawAxis(const Range& range) {
    if (_show_axis.x)
        cv::line(_image, _normalize({0, range.first.y}, range), _normalize({0, range.second.y}, range), cv::Scalar(255, 255, 255));

    if (_show_axis.y)
        cv::line(_image, _normalize({range.first.x, 0}, range), _normalize({range.second.x, 0}, range), cv::Scalar(255, 255, 255));
}

void Graph::_update() {
    // Exit early if there is no data
    if (_data.size() == 0 || _data[0].data.size() == 0)
        return;

    // Exit early if holding
    if (_holding)
        return;

    // Clear the window
    _image.setTo(cv::Scalar(0, 0, 0));

    // Get graph range for normalization
    Range range = _dynamicRange();

    // Plot axis
    _drawAxis(range);

    // Plot normalized data
    for (const DataSet& d : _data) {
        // Normalize data
        std::vector<cv::Point> points;
        for (const Point2<double>& p : d.data)
            points.push_back(_normalize(p, range));

        // Plot in the window
        for (const cv::Point& p : points)
            cv::circle(_image, {p.x, (int) _size.y - p.y}, 1, d.color, 1);
    }

    // Update window
    cv::imshow(_NAME, _image);
    cv::waitKey(1);
}

Graph::Graph(std::string name): _NAME(name) {
    cv::namedWindow(_NAME, cv::WINDOW_KEEPRATIO);
    _resizeWindow(_size);
}

void Graph::resize(size_t size_x, size_t size_y) {
    resize({size_x, size_y});
}

void Graph::resize(Point2<size_t> size) {
    _resizeWindow(size);
    _size = size;
    _update();
}

void Graph::setMargin(size_t margin) {
    _margin = margin;
    _update();
}

void Graph::holdOn() {
    _holding = true;
}

void Graph::holdOff() {
    _holding = false;
    _update();
}

void Graph::showAxes() {
    _show_axis = {true, true};
}

void Graph::showXAxis() {
    _show_axis.x = true;
}

void Graph::showYAxis() {
    _show_axis.y = true;
}

void Graph::hideAxes() {
    _show_axis = {false, false};
}

void Graph::hideXAxis() {
    _show_axis.x = false;
}

void Graph::hideYAxis() {
    _show_axis.y = false;
}

void Graph::fixedRange(const Point2<double>& x, const Point2<double>& y) {
    _range = {{x.x, y.x}, {x.y, y.y}};
}

void Graph::dynamicRange() {
    _range = {DYNAMIC_RANGE, DYNAMIC_RANGE};
}

void Graph::plot(const std::vector<double>& x, const std::vector<double>& y, cv::Scalar color) {
    // Ensure data vectors are the same size
    if (x.size() != y.size())
        throw std::invalid_argument("Cannot plot data sets of two different sizes");

    // Convert to vector of points
    std::vector<Point2<double>> points;
    for (size_t i = 0; i < x.size(); i++)
        points.push_back({x[i], y[i]});

    // Plot these points
    plot(points, color);
}

void Graph::plot(const std::vector<Point2<double>>& data, cv::Scalar color) {
    // Exit early if there are no points being plotted
    if (data.size() == 0)
        return;

    // Store data
    _data.push_back({data, color});

    // Update plot
    _update();
}

void Graph::clear() {
    _data = {};
    _update();
}