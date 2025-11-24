#include <random>
#include <vector>
#include <cmath>

#include "Types/Point2.hpp"
#include "Visuals/Graph.h"

int main() {
    // Create and scale the graph
    Graph graph;
    graph.resize(500, 500);

    graph.dynamicRange();

    // Multiplier for the quadratic
    double mult = 0;

    while (true) {
        // Hold updates
        graph.holdOn();

        // Clear what's currently plotted
        graph.clear();

        // Store points to be plotted in vectors
        std::vector<Point2<double>> pts1, pts2;
        for (double i = -5; i < 5; i += 0.1) {
            pts1.push_back({
                i, i * i * i
            });

            pts2.push_back({
                i, mult * i * i
            });
        }
        
        // Plot these vectors of data
        graph.plot(pts1, cv::Scalar(0, 255, 255));  // Yellow cubic
        graph.plot(pts2, cv::Scalar(0, 0, 255));    // Red quadratic

        // Release updates
        graph.holdOff();

        // Update multiplier
        mult = std::fmod(mult + 0.01, 10);
    }
        
    return 0;
}