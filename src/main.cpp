#include "../include/Motion/Mallet.h"
#include "../include/Motion/Puck.h"
#include <iostream>
#include <fstream>
#include <cmath>

Mallet mallet;
Puck puck;

// Sample rate and puck angle
double sample_time = 1.0 / 90;
double angle = 226;

// Mallet location data
Point2<double> mallet_pos(13.5, 7);

// Puck location data
Point2<double> puck_prev_pos(12, 42);
Point2<double> puck_cur_pos = puck_prev_pos +
    Constants::MAX_PUCK_SPEED * sample_time * Point2<double>(std::cos(angle * M_PI / 180.0), std::sin(angle * M_PI / 180.0));

// Point count
size_t num_points = 20;

int main() {
    // Initialize puck
    puck.readPosition(puck_prev_pos, sample_time);
    puck.readPosition(puck_cur_pos, sample_time);

    // Initialize mallet
    mallet.readPosition(mallet_pos);

    // Run calculations
    Matrix<Point3<double>> sample_data = puck.estimateTrajectory(num_points);

    // Determine time for mallet to reach these points
    Matrix<Point2<double>> puck_pos(num_points);
    Matrix<double> relative_times(num_points);
    Matrix<double> mallet_times(num_points);
    Matrix<double> puck_times(num_points);

    size_t i = 0;
    for (auto p : sample_data) {
        double mallet_time = mallet.timeToReach({p.x, p.y});

        relative_times(i) = p.z - mallet_time;
        mallet_times(i) = mallet_time;
        puck_pos(i) = {p.x, p.y};
        puck_times(i) = p.z;

        i++;
    }

    // Output positions
    std::cout << puck_pos.transpose() << std::endl;
        
    // Open MATLAB file
    std::ofstream ml;
    ml.open("Matlab.m");

    if (!ml.is_open()) {
        std::cerr << "Could not open file\n";
        return -1;
    }

    ml << "X = " << puck_times << ";\n";
    ml << "Y = " << relative_times << ";\n";
    ml << "zero = " << puck_times * 0 << ";\n";

    ml << "figure(1)\n";
    ml << "plot(X, Y, X, zero, X, Y, \".\")\n";

    ml << "xlabel('Time (seconds)');\n";
    ml << "ylabel('Puck Arrival Time - Mallet Arrival Time (seconds)');\n";
    ml << "title('Relative Arrival Times Over Time');\n";

    ml.close();

    return 0;
}