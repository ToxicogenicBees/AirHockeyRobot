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
    PinDef(uint8_t pin = 0, uint8_t mode = 0);

    /**
     * @brief Reads the current state of the pin
     * 
     * @return The current state of the pin
     */
    bool read() const;

    /**
     * @brief Writes the desired state to the pin
     * 
     * @param state The desired state
     */
    void write(bool state);

    /**
     * @brief Toggles the pin's current state
     */
    void toggle();

    /**
     * @brief Initializes the pin's mode, required to read pin properly
     */
    void init();
    
    /**
     * @brief Assignment operator overload
     * 
     * @param state The desired state
     */
    PinDef& operator=(bool state);

    /**
     * @brief Returns this->read() when converted to boolean
     */
    operator bool() const;
};