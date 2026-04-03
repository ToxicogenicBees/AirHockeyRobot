#include "Sensors/TemperatureSensor.hpp"

#include <cmath>

namespace {
    // TMP6131LPGM thermistor
    // Transfer function for temperature vs. Vsense voltage
    // is given in the TI Thermistor design tool
    constexpr double THRM[5]
        = { -2.885698e2, 1.556236e2, 7.191258e1, -5.134061e1, 1.244030e1 };

    // ADC convertion ratio
    constexpr double ADC_BIAS = 3.3f / 4095;

    // Sampling
    constexpr double TEMP_ERR = 5.0;
    constexpr uint8_t SAMPLES = 20;
}

TemperatureSensor::TemperatureSensor(PinDef& read) : Sensor(read) {}

double TemperatureSensor::_sample() {
    // STM32 has 12-bit ADC, 3.3 logic
    uint16_t adc_read = _pins[0]->readAnalog();

    // Voltage over the thermistor
    double v_sense = ADC_BIAS * adc_read;

    // 4th order regression to get temperature
    double v_sense_exp = 1;
    double temp = THRM[0];
    for (size_t i = 1; i < 5; i++) {
        v_sense_exp *= v_sense;
        temp += THRM[i] * v_sense_exp;
    }
    return temp;
}

double TemperatureSensor::temperature() {
    // Sample temperature
    double temp = _sample();

    // If current reading is within tolerance, add to buffer and return
    if (std::fabs(temp - _prev_avg) < TEMP_ERR) {
        // Store temp in buffer
        _buffer[_buf_ind] = temp;
        _buf_ind = (_buf_ind + 1) % _SAMPLES;

        // Calculate average of buffer with new data
        double avg = 0;
        for (size_t i = 0; i < _SAMPLES; i++)
            avg += _buffer[i];
        avg /= _SAMPLES;

        // Store and return average
        _prev_avg = avg;
        return avg;
    }
    
    // Return previous average
    return _prev_avg;
}

void TemperatureSensor::init() {
    // Default initialization
    Sensor::init();

    // Fill the buffer with readings
    _prev_avg = _sample();
    for (size_t i = 0; i < _SAMPLES; i++)
        _buffer[i] = _prev_avg;
}
