#include "../include/Motion/Mallet.h"
#include "../include/Motion/Puck.h"
#include <iostream>
#include <fstream>

Mallet mallet;
Puck puck;

// Mallet location data
Point2<double> mallet_pos(21.8, 7.2);

// Puck location data
Point2<double> puck_prev_pos(19.2, 33.6);
Point2<double> puck_cur_pos(18.1230011,33.410096);
double sample_time = 1.0 / 90;

// Sample rate
size_t num_points = 2000;

int main() {
    // Initialize puck
    puck.readPosition(puck_prev_pos, sample_time);
    puck.readPosition(puck_cur_pos, sample_time);

    // Initialize mallet
    mallet.readPosition(mallet_pos);

    // Run calculations
    Matrix<Point3<double>> sample_data = puck.estimateTrajectory(num_points);

    // Determine time for mallet to reach these points
    Matrix<double> relative_times(num_points);
    Matrix<double> mallet_times(num_points);
    Matrix<double> puck_times(num_points);

    size_t i = 0;
    for (auto p : sample_data) {
        double mallet_time = mallet.timeToReach({p.x, p.y});

        relative_times(i) = p.z - mallet_time;
        mallet_times(i) = mallet_time;
        puck_times(i) = p.z;

        i++;
    }
        
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
    ml << "plot(X, Y, X, zero)\n";

    ml.close();

    return 0;
}