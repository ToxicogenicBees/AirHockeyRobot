#include <algorithm>
#include <stdexcept>

#include "Visuals/Graph.h"

Graph::Graph() {
    _window = cv::Mat(_size.y, _size.x);
}

void Graph::resize(Point2<size_t> size, size_t margin) {
    _margin = margin;
    _size = size;

    plot(data.x, data.y);
}

void Graph::resize(Point2<size_t> size) {
    _size = size;

    plot(data.x, data.y);
}

void Graph::setMargin(size_t margin) {
    _margin = margin;

    plot(data.x, data.y);
}

void Graph::plot(const std::vector<Point2<double>>& x, const std::vector<Point2<double>>& y) {
    if (x.size() != y.size())
        throw std::invalid_argument("Cannot plot data sets of two different sizes");

    // Exit early if there are no points being plotted
    if (x.size() == 0 || y.size() == 0)
        return;

    // Find data range
    double min_x = std::min(x);
    double max_x = std::max(x);
    double min_y = std::min(y);
    double max_y = std::max(y);

    // Normalized window points
    std::vector<cv::Point> points;

    for (size_t i = 0; i < x.size(); i++) {
        points.push_back({
            _margin + (min_x + x[i]) / (max_x - min_x - 2 * _margin),
            _margin + (min_y + y[i]) / (max_y - min_y - 2 * _margin)
        });
    }

    // Plot points
    _window.setTo(cv::Scalar(0, 0, 0));

    for (const cv::Point& p : points)
        cv::circle(_window, {p.x, _size.y - p.y}, 2, cv::Scalar(255, 255, 255), 2);

    // Update window
    cv::imshow("Graph", _window);
    cv::waitKey(1);
}

void Graph::plot(const std::vector<Point2<double>>& data) {
    std::vector<double> x, y;

    for (auto p : data) {
        x.push_back(p.x);
        y.push_back(p.y);
    }

    plot(x, y);
}