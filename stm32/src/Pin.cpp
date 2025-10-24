#include "Pin.h"

Pin::Pin(GPIO_TypeDef* port, uint16_t pin)
    : PORT(port), PIN(pin) {}

bool Pin::read() const {
    return HAL_GPIO_ReadPin(PORT, PIN) == GPIO_PIN_SET;
}

void Pin::write(const GPIO_PinState& state) {
    HAL_GPIO_WritePin(PORT, PIN, state);
}

void Pin::write(bool state) {
    HAL_GPIO_WritePin(PORT, PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Pin::toggle() {
    HAL_GPIO_TogglePin(PORT, PIN);
}

void Pin::init(uint32_t mode, uint32_t pull, uint32_t speed, uint32_t alternate) {
    GPIO_InitTypeDef GPIO_InitStruct = { PIN, mode, pull, speed, alternate };
    HAL_GPIO_Init(PORT, &GPIO_InitStruct);
}

Pin& Pin::operator=(const GPIO_PinState& state) {
    write(state);
    return *this;
}

Pin& Pin::operator=(bool state) {
    write(state);
    return *this;
}

Pin::operator bool() const {
    return read();
}