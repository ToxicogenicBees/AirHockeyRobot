#pragma once

#include <Arduino.h>
#include <stdint.h>

struct PinDef {
    // Pin and mode values
    const uint8_t PIN, MODE;

    /**
     * @brief Creates a new pin definition
     * 
     * @param pin   The pin number
     * @param mode  The Arduino.h pin mode
     */
    PinDef(uint8_t pin = 0, uint8_t mode = 0)
        : PIN(pin), MODE(mode) {}

    /**
     * @brief Reads the current (digital) state of the pin
     * 
     * @return The current (digital) state of the pin
     */
    uint8_t read() const {
        return digitalRead(PIN) == HIGH;
    }

    /**
     * @brief Reads the current (analog) state of the pin
     * 
     * @return The current (analog) state of the pin
     */
    uint32_t readAnalog() const {
        return analogRead(PIN);
    }

    /**
     * @brief Writes the desired state to the pin
     * 
     * @param state The desired state
     */
    void write(uint8_t state) {
        digitalWrite(PIN, state);
    }

    /**
     * @brief Initializes the pin's mode, required to read pin properly
     */
    void init() {
        pinMode(PIN, MODE);
    }
};
