#pragma once

#include <opencv2/opencv.hpp>
#include <limits>
#include <vector>

#include "Types/Point2.hpp"

struct DataSet {
    std::vector<Point2<double>> data;
    cv::Scalar color;
};

class Graph {
    using Range = std::pair<Point2<double>, Point2<double>>;

    public:
        static const Point2<double> DYNAMIC_RANGE;

    private:
        const std::string _NAME;                // Name of the graph window

        std::vector<DataSet> _data;             // Plot data
        
        Point2<size_t> _size = {100, 100};      // Window size
        size_t _margin = 10;                    // Window margin
        cv::Mat _image;                         // Window image

        Range _range = {{-5, -5}, {5, 5}};      // Data range

        Point2<bool> _show_axis = {true, true}; // Flags to show each axis
        bool _holding = false;                  // Flag to hold off on updating the graph

        // Normalize a point to a range
        cv::Point _normalize(const Point2<double>& point, const Range& range);

        // Resize the graph window
        void _resizeWindow(Point2<size_t> size);

        // Draw the coordinate axes
        void _drawAxis(const Range& range);

        // Find the dynamic range of the data
        Range _dynamicRange();

        // Update the graph image
        void _update();

    public:
        /***
         * @brief Create a new graph object
         * 
         * @param name  The name of the graph
         */
        Graph(std::string name = "Graph");

        /***
         * @brief Resize the graph size
         * 
         * @param size_x    The image width, in pixels
         * @param size_y    The image length, in pixels
         */
        void resize(size_t size_x, size_t size_y);

        /***
         * @brief Resize the graph size
         * 
         * @param size  The image size, in pixels
         */
        void resize(Point2<size_t> size);

        /***
         * @brief Set the graph's margin size
         * 
         * @param margin    The margin size, in pixels
         */
        void setMargin(size_t margin);

        /***
         * @brief Stops the graph from updating uuntil holdOff() is called
         */
        void holdOn();

        /***
         * @brief Resumes updating the graph
         */
        void holdOff();

        /***
         * @brief Shows both the x- and y-axis
         */
        void showAxes();

        /***
         * @brief Shows the x-axis
         */
        void showXAxis();

        /***
         * @brief Shows the y-axis
         */
        void showYAxis();

        /***
         * @brief Hidess both the x- and y-axis
         */
        void hideAxes();

        /***
         * @brief Hides the x-axis
         */
        void hideXAxis();

        /***
         * @brief Hides the y-axis
         */
        void hideYAxis();

        /**
         * @brief Sets a range for the x- and y-axis
         * 
         * @param x The x-axis range
         * @param y The y-axis range
         */
        void fixedRange(const Point2<double>& x, const Point2<double>& y);

        /**
         * @brief Sets a dynamic range for the x-y and y-axis
         */
        void dynamicRange();

        /**
         * @brief Adds a new set of data to the graph
         * 
         * @param x     The x-axis data
         * @param y     The y-axis data
         * @param color The color of this plot (defaults to white)
         */
        void plot(const std::vector<double>& x, const std::vector<double>& y, cv::Scalar color = {255, 255, 255});

        /**
         * @brief Adds a new set of data to the graph
         * 
         * @param data  The data being plotted
         * @param color The color of this plot (defaults to white)
         */
        void plot(const std::vector<Point2<double>>& data, cv::Scalar color = {255, 255, 255});

        /**
         * @brief Clears all data from the graph
         */
        void clear();

        /**
         * @brief Returns the name of the graph
         * 
         * @return The name of the graph
         */
        std::string name() const { return _NAME; }

        /**
         * @brief Returns the size of the graph
         * 
         * @return The size of the graph
         */
        Point2<size_t> size() const { return _size; }

        /**
         * @brief Returns the margin size of the graph
         * 
         * @return The margin size of the graph
         */
        size_t margin() const { return _margin; }

        /**
         * @brief Returns the x-range of the graph
         * 
         * @return The x-range of the graph
         */
        Point2<double> rangeX() const { return _range.first; }

        /**
         * @brief Returns the y-range of the graph
         * 
         * @return The y-range of the graph
         */
        Point2<double> rangeY() const { return _range.first; }

        /**
         * @brief Returns the range of the graph
         * 
         * @return The range of the graph
         */
        Range range() const { return _range; }
};