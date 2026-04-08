#include "Routines/MotionTestRoutine.hpp"
#include "Constants.hpp"

#include <vector>

namespace {
    constexpr double TIME_BETWEEN_UPDATES = 2;
    const std::vector<Point2<double>> TARGETS = {
        Constants::Mallet::DODGE_LEFT, Constants::Mallet::DODGE_RIGHT, Constants::Mallet::HOME
    };
}

void MotionTestRoutine::updateTarget() {
    auto seconds = _timer.delta<std::chrono::seconds>();

    if (seconds >= TIME_BETWEEN_UPDATES * TARGETS.size()) {
        _timer.reset();
        seconds = 0;
    }

    softTransmit(TARGETS[(int)(seconds / TIME_BETWEEN_UPDATES)]);
    softTransmit({0, 0, 800, 800});
}
