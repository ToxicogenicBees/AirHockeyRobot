#pragma once

#include <Arduino.h>
#include <stdint.h>

struct Pin {
    uint8_t PIN, MODE;

    Pin(uint32_t pin = 0, uint32_t mode = 0);

    bool read() const;

    void write(bool state);

    void toggle();

    void init();
    
    Pin& operator=(bool state);

    operator bool() const;
};