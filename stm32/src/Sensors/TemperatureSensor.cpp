#include "Sensors/TemperatureSensor.h"

TemperatureSensor::TemperatureSensor(PinDef& read) : Sensor(1, &read) {}

float TemperatureSensor::transferFunctionTempVsVsense(float Vsense) {
    // we are using TMP6131LPGM thermistor
    // the transfer function for temperature vs. Vsense voltage
    // is given in the TI Thermistor design tool
    float THRM_A0 =	-2.885698E+02	;
    float THRM_A1 =	1.556236E+02	;
    float THRM_A2 = 7.191258E+01	;
    float THRM_A3 = -5.134061E+01	;
    float THRM_A4 = 1.244030E+01	;

    // 4th order regression to get temperature	
    float THRM_TEMP = (THRM_A4 * powf( Vsense,4)) + (THRM_A3 * powf( Vsense,3)) + (THRM_A2 * powf( Vsense,2)) + (THRM_A1 *  Vsense) + THRM_A0;								
    return THRM_TEMP;									
}

float TemperatureSensor::temperature() {
    int twelveBitADCRead = analogRead(_PINS[0]->PIN); // stm32 has 12-bit ADC, 3.3 logic
    float Vsense = twelveBitADCRead/4095 * 3.3; // voltage over the thermistor

    return transferFunctionTempVsVsense(Vsense);
}