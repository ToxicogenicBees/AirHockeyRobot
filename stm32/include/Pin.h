#pragma once

#include "stm32f4xx_hal.h"

struct Pin {
    GPIO_TypeDef* PORT;
    uint16_t PIN;

    Pin(GPIO_TypeDef* port = GPIOA, uint16_t pin = GPIO_PIN_0);

    bool read() const;

    void write(const GPIO_PinState& state);

    void write(bool state);

    void toggle();

    void init(
        uint32_t mode = GPIO_MODE_OUTPUT_PP,
        uint32_t pull = GPIO_NOPULL,
        uint32_t speed = GPIO_SPEED_FREQ_LOW,
        uint32_t alternate = 0
    );

    Pin& operator=(const GPIO_PinState& state);
    
    Pin& operator=(bool state);

    operator bool() const;
};