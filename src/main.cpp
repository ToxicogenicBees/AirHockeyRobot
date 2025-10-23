#include "../include/Motion/Mallet.h"
#include "../include/Motion/Puck.h"
#include <iostream>
#include <fstream>

Mallet mallet;
Puck puck;



// Mallet location data
Point2<double> mallet_pos(22, 6);

// Puck location data
Point2<double> puck_prev_pos(20.8, 36.4);
Point2<double> puck_cur_pos(19.9041644,35.7727292);
double sample_time = 1.0 / 90;

const double SAMPLE_RATE = 1.0 / 90;
const size_t NUM_POINTS = 20;

void initTable(const Point2<double>& mallet_pos, const Point2<double>& puck_pos, double puck_angle) {
    mallet.readPosition(mallet_pos);

    puck.readPosition(puck_pos, SAMPLE_RATE);
}

int main() {
    // Initialize the mallet and puck
    initTable({22.0, 6.0}, {20.8, 36.4}, 270);

    // Run calculations
    Matrix<Point3<double>> sample_data = puck.estimateTrajectory(NUM_POINTS);

    // Determine time for mallet to reach these points
    Matrix<double> relative_times(NUM_POINTS);
    Matrix<double> mallet_times(NUM_POINTS);
    Matrix<double> puck_times(NUM_POINTS);

    Matrix<Point2<double>> pts(NUM_POINTS);
    size_t j = 0;
    for (auto p : sample_data) {
        pts(j++) = Point2<double>(p.x, p.y);
    }
    std::cout << pts.transpose() << std::endl;

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
    ml << "plot(X, Y, X, zero, X, Y, \".\")\n";

    ml.close();

    return 0;
}