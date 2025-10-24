#pragma once

#include "PinDef.h"

class Motor {
    private:

    public:
        /**
         * @brief Creates a new motor
         */
        Motor();

        /**
         * @brief Initializes a motor and its pins
         */
        void init();
};