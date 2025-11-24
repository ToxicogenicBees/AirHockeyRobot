#include <opencv2/opencv.hpp>
#include <windows.h>
#include <chrono>
#include <thread>

#include "Examples/Physics.hpp"
#include "Visuals/Table.h"
#include "Motion/Mallet.h"
#include "Motion/Puck.h"
#include "Constants.h"

Point2<double> myPointsDataTest[] = {{1, 2}, {5, 6}, {7, 9}, {10, 13}, {11, 15}}; 
Point2<double> *myPtr = myPointsDataTest;

void PHYSICS_STEP() {
    // Update mallet position from sensor data
    Point2<double> prev_pos = *myPtr;
    Point2<double> cur_pos = *(++myPtr);

    Mallet::orient(cur_pos, (cur_pos - prev_pos) / Constants::SAMPLE_PERIOD);
    
    // Step physics
    Physics::step();
    
    // Pause for a sample tick
    std::this_thread::sleep_for(std::chrono::microseconds(Constants::SAMPLE_PERIOD));
}

int main() {
    // Initialize the table
    Puck::orient(Constants::Puck::HOME, Constants::Puck::SPEED * Point2<double>(0.5 * std::sqrt(2), 0.5 * std::sqrt(2)));
    Mallet::orient(Constants::Mallet::HOME);

    // Run physics in the background
    std::thread physics_step(PHYSICS_STEP);
    physics_step.detach();

    // Render processing
    while (true) {
        Table::render();
    }

    return 0;
}