#include <opencv2/opencv.hpp>
#include <algorithm>
#include <windows.h>
#include <chrono>
#include <thread>

#include "Simulation/Physics.hpp"
#include "Simulation/Table.h"
#include "Motion/Mallet.h"
#include "Motion/Puck.h"
#include "Constants.h"

#include "Motion/Routines/BasicDefenseRoutine.h"
#include "Motion/Routines/MotionTestRoutine.h"
#include "Motion/Routines/StrikeTestRoutine.h"

void PHYSICS_STEP() {
    while (true) {
        // Step physics
        Physics::step();
    
        // Pause for a sample tick
        std::this_thread::sleep_for(std::chrono::microseconds(Constants::SAMPLE_PERIOD));
    }
}

void MALLET_UPDATE() {
    while (true) {
        Mallet::updateTarget();
    }
}

int main() {
    // Initialize the table
    Puck::orient({Constants::Puck::HOME, Constants::Puck::SPEED * Point2<double>(0.5 * std::sqrt(2), 0.5 * std::sqrt(2))});
    Mallet::orient({Constants::Mallet::HOME, Point2<double>::zero()});

    BasicDefenseRoutine routine;
    Mallet::setRoutine(routine.clone());

    // Run physics in the background
    std::thread mallet_update(MALLET_UPDATE);
    std::thread physics_step(PHYSICS_STEP);
    mallet_update.detach();
    physics_step.detach();

    // Render processing
    while (true) {
        Table::render();
    }

    return 0;
}