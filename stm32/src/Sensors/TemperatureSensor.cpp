#include "Sensors/TemperatureSensor.h"

#include <cmath>

TemperatureSensor::TemperatureSensor(PinDef& read) : Sensor<1>(&read) {}

double TemperatureSensor::_sample() {
    // STM32 has 12-bit ADC, 3.3 logic
    uint16_t adc_read = _pins[0]->readAnalog();

    // Voltage over the thermistor
    double v_sense = _ADC_BIAS * adc_read;

    // 4th order regression to get temperature
    double v_sense_exp = 1;
    double temp = _THRM[0];
    for (size_t i = 1; i < 5; i++) {
        v_sense_exp *= v_sense;
        temp += _THRM[i] * v_sense_exp;
    }
    return temp;
}

double TemperatureSensor::temperature() {
    // Sample temperature
    double temp = _sample();

    // If current reading is within tolerance, add to buffer and return
    if (std::fabs(temp - _prev_avg) < _TEMP_ERR) {
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
    _prev_avg = 0;

    for (size_t i = 0; i < _SAMPLES; i++) {
        _buffer[i] = _sample();
        _prev_avg += _buffer[i];
    }

    _prev_avg /= _SAMPLES;
}