#include <opencv2/opencv.hpp>
#include <algorithm>
#include <windows.h>
#include <chrono>
#include <thread>

#include "Examples/Physics.hpp"
#include "Visuals/Table.h"
#include "Motion/Mallet.h"
#include "Motion/Puck.h"
#include "Constants.h"

int main() {
    // Initialize the table
    Puck::orient(Constants::Puck::HOME, Constants::Puck::SPEED * Point2<double>(0.5 * std::sqrt(2), 0.5 * std::sqrt(2)));
    Mallet::orient(Constants::Mallet::HOME);

    // Render in the background
    std::thread rendering = Table::render();

    // Run physics processing
    while (true) {
        // Step physics
        Physics::step();

        // Pause for a sample tick
        std::this_thread::sleep_for(std::chrono::microseconds(Constants::SAMPLE_PERIOD));
    }

    return 0;
}