#include "Sensors/TemperatureSensor.h"

#include <cmath>

TemperatureSensor::TemperatureSensor(PinDef& read) : Sensor(1, &read) {}

float TemperatureSensor::_transferFunctionTempVsVsense(float v_sense) {
    // 4th order regression to get temperature	
    return _THRM_A4 * std::pow(v_sense, 4)
        + _THRM_A3 * std::pow(v_sense, 3)
        + _THRM_A2 * std::pow(v_sense, 2)
        + _THRM_A1 * v_sense
        + _THRM_A0;									
}

float TemperatureSensor::temperature() {
    uint16_t adc_read = _PINS[0]->readAnalog();     // STM32 has 12-bit ADC, 3.3 logic
    float v_sense = 3.3f * (adc_read / 4095.0f);    // Voltage over the thermistor

    return _transferFunctionTempVsVsense(v_sense);
}