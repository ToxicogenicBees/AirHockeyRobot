#ifndef TIMER_HPP
#define TIMER_HPP

#include <stdint.h>
#include <chrono>

class Timer {
    private:
        using Clock = std::chrono::high_resolution_clock;
        using SampleTime = std::chrono::time_point<Clock>;
        
        SampleTime _prev_sample;

    public:
        Timer() { reset(); }

        void reset() { _prev_sample = Clock::now(); }

        int64_t delta() {
            SampleTime now = Clock::now();
            return std::chrono::duration_cast<std::chrono::microseconds>(now - _prev_sample).count();
        }
};

#endif
