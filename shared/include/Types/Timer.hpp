#pragma once

#include <stdint.h>
#include <chrono>

class Timer {
    private:
        using Clock = std::chrono::high_resolution_clock;
        using SampleTime = std::chrono::time_point<Clock>;
        
        SampleTime _prev_sample;

    public:
        /**
         * @brief   Creates a new timer
         */
        Timer() {
            reset();
        }

        /**
         * @brief   Sets the clocks timestamp to the current OS clock value
         */
        void reset() {
            _prev_sample = Clock::now();
        }

        /**
         * @brief   Fetches the amount of time passed since the clock was last reset
         * 
         * @param   cast    The std::chrono::duration_cast type to use when counting the time passed
         *                  Default is std::chrono::microseconds
         * 
         * @return  The amount of time passed since the clock was last reset
         */
        template<class cast_type = std::chrono::microseconds>
        int64_t delta() const {
            return std::chrono::duration_cast<cast_type>(Clock::now() - _prev_sample).count();
        }
};
