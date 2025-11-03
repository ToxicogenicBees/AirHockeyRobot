#include "Sensors/TemperatureSensor.h"

#include <cmath>

TemperatureSensor::TemperatureSensor(PinDef& read) : Sensor(1, &read) {}

double TemperatureSensor::_sample() {
    uint16_t adc_read = _PINS[0]->readAnalog();     // STM32 has 12-bit ADC, 3.3 logic
    double v_sense = _ADC_BIAS * adc_read;           // Voltage over the thermistor

    // 4th order regression to get temperature
    return _THRM_A4 * std::pow(v_sense, 4)
        + _THRM_A3 * std::pow(v_sense, 3)
        + _THRM_A2 * std::pow(v_sense, 2)
        + _THRM_A1 * v_sense
        + _THRM_A0;
}

double TemperatureSensor::_sampleBurst(uint8_t samples) {
    double sum = 0;

    for (size_t i = 0; i < samples; i++)
        sum += _sample();

    return sum / samples;
}

double TemperatureSensor::temperature() {
    return _sampleBurst(_SAMPLES);
}