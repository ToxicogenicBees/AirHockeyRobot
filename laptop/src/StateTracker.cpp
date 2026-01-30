#include "StateTracker.h"
#include <iostream>
#include <ctype.h>

uint8_t StateTracker::_difficulty = 0;
bool StateTracker::_state = false;

void StateTracker::init() {
    // Fetch difficulty from console
    char input = ' ';

    while (1) {
        std::cout << "Choose a desired difficulty: ";
        std::cin >> input;

        if (isdigit(input))
            break;
        else
            std::cout << "Difficulty must be an integer 0 - 9\n";
    }

    // Store difficulty as number
    _difficulty = input - '0';
}

void StateTracker::enable() {
    _state = true;
}

void StateTracker::disable() {
    _state = false;
}