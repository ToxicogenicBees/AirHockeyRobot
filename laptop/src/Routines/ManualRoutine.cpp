#include <windows.h>
#include <algorithm>

#include "Routines/ManualRoutine.hpp"
#include "Motion/Table.hpp"
#include "Constants.hpp"

namespace {
    constexpr double POSITION_CHANGE_PER_SECOND = 10;    // Inches

    bool pressed(char key) {
        return GetAsyncKeyState(key) & 0x8000;
    }
}

void ManualRoutine::updateTarget() {

    // Get time difference
    double dt = 1e-6 * _timer.delta<std::chrono::microseconds>();
    _timer.reset();
        
    if (pressed('w') || pressed('W'))
        _position.y += POSITION_CHANGE_PER_SECOND * dt;
    if (pressed('a') || pressed('A'))
        _position.x -= POSITION_CHANGE_PER_SECOND * dt;
    if (pressed('s') || pressed('S'))
        _position.y -= POSITION_CHANGE_PER_SECOND * dt;
    if (pressed('d') || pressed('D'))
        _position.x += POSITION_CHANGE_PER_SECOND * dt;

    // Clamp mallet position to it's movement range
    _position = {
        std::clamp(_position.x, Constants::Mallet::LIMIT_BL.x + 1, Constants::Mallet::LIMIT_TR.x - 1),
        std::clamp(_position.y, Constants::Mallet::LIMIT_BL.y + 1, Constants::Mallet::LIMIT_TR.y - 1)
    };

    // Transmit position
    softTransmit({0, 0, 350, 350});
    softTransmit(_position);
}
