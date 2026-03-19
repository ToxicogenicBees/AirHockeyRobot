#ifndef STATETRACKER_H
#define STATETRACKER_H

#include <stdint.h>

class StateTracker {
    private:
        static int _difficulty;
        static bool _state;

    public:
        /**
         * @brief Initializes the difficulty through console input
         */
        static void init();

        /**
         * @brief Enables gantry motion
         */
        static void enable();

        /**
         * @brief Disables gantry motion
         */
        static void disable();

        /**
         * @brief Fetches the current difficulty
         * 
         * @return The current difficulty
         */
        static int getDifficulty() { return _difficulty; }

        /**
         * @brief Fetches the current state
         * 
         * @return The current state
         */
        static uint8_t getState() { return _state; }
};

#endif
