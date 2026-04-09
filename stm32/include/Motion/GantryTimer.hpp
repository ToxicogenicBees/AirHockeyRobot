#pragma once

#include <Arduino.h>
#include <functional>

class GantryTimer {
    private:
        typedef void (*Callback)();

        // HAL hardware timer
        TIM_TypeDef* _tim;
        HardwareTimer* _hw_timer;

        // Interrupt callback
        Callback _interrupt_callback;

    public:
        /**
         * @brief Create a new gantry timer
         * 
         * @param timer The desired hardware timer
         */
        GantryTimer(TIM_TypeDef* tim) : _tim(tim), _hw_timer(new HardwareTimer(tim)) {}

        /**
         * @brief Destroy a gantry timer
         */
        ~GantryTimer() {
            delete _hw_timer;
        }

        /**
         * @brief Initialize the timer
         * 
         * @param interrupt_callback    Callback for each rising clock edge
         */
        void init(Callback interrupt_callback) {
            // https://github.com/stm32duino/Arduino_Core_STM32/wiki/HardwareTimer-library
            _hw_timer->setMode(1, TIMER_OUTPUT_DISABLED, 0);  // no pin output, only for interrupt
            _hw_timer->pause();
            _hw_timer->attachInterrupt(interrupt_callback);
            _hw_timer->setCount(0);
            _hw_timer->setPrescaleFactor((_hw_timer->getTimerClkFreq() / 1000000) - 1);
        }

        /**
         * @brief Start the clock
         */
        void start() {
            _tim->CNT = 0;
            _tim->CR1 |= TIM_CR1_CEN;
        }

        /**
         * @brief Stop the clock
         */
        void stop() {
            _tim->CR1 &= ~TIM_CR1_CEN;
            _tim->CNT = 0;
        }

        /**
         * @brief Set the period of the clock
         * 
         * @param period    The period of the clock
         */
        void setPeriod(uint16_t period) {
            _tim->ARR = period;
        }

        /**
         * @brief Get the clock count
         * 
         * @return The clock count
         */
        uint32_t getCount() {
            return _hw_timer->getCount();
        }
};