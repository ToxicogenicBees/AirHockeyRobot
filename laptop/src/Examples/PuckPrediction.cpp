#include "Motion/Mallet.h"
#include "FileFormat.hpp"
#include "Motion/Puck.h"
#include <iostream>
#include <fstream>
#include <cmath>

void initTable(const Point2<double>& mallet_pos, const Point2<double>& puck_pos, double puck_angle) {
    // Calculate the desired puck velocity
    double rads = puck_angle * M_PI / 180;
    Point2<double> vel = Point2<double>(std::cos(rads), std::sin(rads)) * Constants::Puck::SPEED;

    // Initialize moving objects
    Puck::orient(puck_pos, vel);
    Mallet::orient(mallet_pos);
}

int main() {
    // Initialize the mallet and puck
    initTable({14.0, 12.0}, {20.8, 36.4}, 226);

    // Run calculations
    Matrix<Point3<double>> sample_data = Puck::estimateTrajectory();
    std::cout << sample_data << std::endl;

    // Determine time for mallet to reach these points
    Matrix<double> relative_times(Constants::NUM_SAMPLE_POINTS);
    Matrix<double> mallet_times(Constants::NUM_SAMPLE_POINTS);
    Matrix<double> puck_times(Constants::NUM_SAMPLE_POINTS);

    size_t i = 0;
    for (auto p : sample_data) {
        double mallet_time = Mallet::timeToReach({p.x, p.y});

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