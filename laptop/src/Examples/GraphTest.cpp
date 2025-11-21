#include <random>
#include <vector>
#include <cmath>

#include "Types/Point2.hpp"
#include "Visuals/Graph.h"

Graph graph;

int main() {
    graph.resize(500, 500);

    double pow = 0;

    while (true) {
        graph.holdOn();

        graph.clear();

        std::vector<Point2<double>> pts1, pts2;
        for (double i = -5; i < 5; i += 0.1) {
            pts1.push_back({
                i, i * i * i
            });

            pts2.push_back({
                i, pow * i * i
            });
        }
        
        graph.plot(pts1, cv::Scalar(0, 255, 255));
        graph.plot(pts2, cv::Scalar(0, 0, 255));

        graph.holdOff();

        pow = std::fmod(pow + 0.01, 5);
    }
        
    return 0;
}