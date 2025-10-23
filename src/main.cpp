#include "../include/Motion/Mallet.h"
#include "../include/FileFormat.hpp"
#include "../include/Motion/Puck.h"
#include <iostream>
#include <fstream>
#include <cmath>

Mallet mallet;
Puck puck;

const double SAMPLE_RATE = 1.0 / 90;
const size_t NUM_POINTS = 20;

void initTable(const Point2<double>& mallet_pos, const Point2<double>& puck_pos, double puck_angle) {
    // Set mallet position
    mallet.readPosition(mallet_pos);

    // Set puck positions
    double rads = puck_angle * M_PI / 180;
    Point2<double> puck_offset(std::cos(rads), std::sin(rads));

    puck.readPosition(puck_pos, SAMPLE_RATE);
    puck.readPosition(puck_pos + Constants::MAX_PUCK_SPEED * SAMPLE_RATE * puck_offset, SAMPLE_RATE);
}

int main() {
    // Initialize the mallet and puck
    initTable({14.0, 12.0}, {20.8, 36.4}, 226);

    // Run calculations
    Matrix<Point3<double>> sample_data = puck.estimateTrajectory(NUM_POINTS);

    // Determine time for mallet to reach these points
    Matrix<double> relative_times(NUM_POINTS);
    Matrix<double> mallet_times(NUM_POINTS);
    Matrix<double> puck_times(NUM_POINTS);

    size_t i = 0;
    for (auto p : sample_data) {
        double mallet_time = mallet.timeToReach({p.x, p.y});

        // Format data matrices
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

    // Store matrices
    ml << ML::matrix("X", puck_times);
    ml << ML::matrix("Y", relative_times);
    ml << ML::matrix("zero", 0 * puck_times);

    // Plot data
    ml << ML::figure(1);
    ml << ML::plot("X", "Y", "X", "zero", ML::points("X", "Y"));

    // Update labels
    ml << ML::xlabel("Time (seconds)");
    ml << ML::ylabel("Puck Arrival Time - Mallet Arrival Time (seconds)");
    ml << ML::title("Relative Arrival Times Over Time");

    // Close file
    ml.close();

    return 0;
}