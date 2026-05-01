#include <windows.h>
#include <algorithm>

#include "Routines/ManualRoutine.hpp"
#include "Motion/Table.hpp"
#include "Constants.hpp"

namespace {
    constexpr double VELOCITY = 10;    // inches / seccond

    bool pressed(char key) {
        return GetAsyncKeyState(key) & 0x8000;
    }
}

void ManualRoutine::updateTarget() {

    // Get time difference
    double dt = 1e-6 * _timer.delta<std::chrono::microseconds>();
    _timer.reset();
    
    // Update position based on key presses
    const auto position_change = VELOCITY * dt;
    if (pressed('w') || pressed('W'))
        _position.y += position_change;
    if (pressed('a') || pressed('A'))
        _position.x -= position_change;
    if (pressed('s') || pressed('S'))
        _position.y -= position_change;
    if (pressed('d') || pressed('D'))
        _position.x += position_change;

    // Clamp mallet position to its movement range
    _position = {
        std::clamp(_position.x, Constants::Mallet::LIMIT_BL.x + 1, Constants::Mallet::LIMIT_TR.x - 1),
        std::clamp(_position.y, Constants::Mallet::LIMIT_BL.y + 1, Constants::Mallet::LIMIT_TR.y - 1)
    };

    // Transmit position
    softTransmit({0.1, 0.1, 250, 400});
    softTransmit(_position);
}
