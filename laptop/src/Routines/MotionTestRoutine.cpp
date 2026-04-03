#include "Routines/MotionTestRoutine.hpp"
#include "Constants.hpp"

#include <vector>

namespace {
    constexpr double TIME_BETWEEN_UPDATES = 2;
    const std::vector<Point2<double>> TARGETS = {
        {5, 5}, {21.5, 5}, Constants::Mallet::HOME
    };
}

MotionTestRoutine::MotionTestRoutine(MovingObject& mallet)
    : Routine(mallet) {}

void MotionTestRoutine::updateTarget() {
    auto seconds = _timer.delta<std::chrono::seconds>();

    if (seconds >= TIME_BETWEEN_UPDATES * TARGETS.size()) {
        _timer.reset();
        seconds = 0;
    }

    _target = TARGETS[(int)(seconds / TIME_BETWEEN_UPDATES)];
    _velocity_profile = VelocityProfile(0, 0, 200, 200);
}
