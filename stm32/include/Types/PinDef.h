#ifndef PINDEF_H
#define PINDEF_H

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
     * @brief Reads the current (digital) state of the pin
     * 
     * @return The current (digital) state of the pin
     */
    uint8_t read() const;

    /**
     * @brief Reads the current (analog) state of the pin
     * 
     * @return The current (analog) state of the pin
     */
    uint32_t readAnalog() const;

    /**
     * @brief Writes the desired state to the pin
     * 
     * @param state The desired state
     */
    void write(uint8_t state);

    /**
     * @brief Writes the desired state to the pin
     * 
     * @param state The desired state
     */
    void writeAnalog(uint32_t state);

    /**
     * @brief Toggles the pin's current (digital) state
     */
    void toggle();

    /**
     * @brief Initializes the pin's mode, required to read pin properly
     */
    void init();

    /**
     * @brief Returns this->read() when converted to uint8_t
     */
    operator uint8_t() const;
};

#endif
