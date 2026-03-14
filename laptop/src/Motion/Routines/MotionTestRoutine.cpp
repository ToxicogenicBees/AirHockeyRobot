#include "Motion/Routines/MotionTestRoutine.h"
#include "Motion/Mallet.h"
#include "Motion/Puck.h"
#include "Constants.h"

#include <vector>

namespace {
    constexpr double TIME_BETWEEN_UPDATES = 2;
    const std::vector<Point2<double>> TARGETS = {
        {5, 5}, {21.5, 5}, Constants::Mallet::HOME
    };
}

MotionTestRoutine::MotionTestRoutine() : Routine() {}

std::unique_ptr<Routine> MotionTestRoutine::clone() const {
    return std::make_unique<MotionTestRoutine>(*this);
}

void MotionTestRoutine::updateTarget() {
    auto seconds = _timer.delta<std::chrono::seconds>();

    if (seconds >= TIME_BETWEEN_UPDATES * TARGETS.size()) {
        _timer.reset();
        seconds = 0;
    }

    _target = TARGETS[(int)(seconds / TIME_BETWEEN_UPDATES)];
}