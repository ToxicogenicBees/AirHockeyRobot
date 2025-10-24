#include "Sensors/DistanceSensor.h"

void delay_us(uint32_t us) {
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000);

    while ((DWT->CYCCNT - start) < ticks) {}
}

void DistanceSensor::_onInterrupt() {
    if (_PINS[1]->read())
        _cur_dist = 0.1715 * (_update_time - HAL_GetTick());
}

DistanceSensor::DistanceSensor(Pin& trig, Pin& echo) : Sensor(2, trig, echo) {
    _registerInterrupt(echo);
}

void DistanceSensor::update() {
    _update_time = HAL_GetTick();

    _PINS[0]->write(false);
    delay_us(2);
    _PINS[0]->write(true);
    delay_us(10);
    _PINS[0]->write(false);
}