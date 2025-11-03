#include "Sensors/TemperatureSensor.h"
#include "Sensors/DistanceSensor.h"
#include "Sensors/LimitSwitch.h"
#include "Comms/SerialLink.h"
#include "Motion/Motor.h"
#include "PinOut.h"

#include <Arduino.h>

// Sensor definitions
DistanceSensor dist_x(dist_x_trig, dist_x_echo);
DistanceSensor dist_y(dist_y_trig, dist_y_echo);
TemperatureSensor temp(temp_read);
LimitSwitch limit(lim);

void setup() {
    // Initialize Serial output
    SerialLink::init();

    // Initialize ADC converter precision
    analogReadResolution(12);

    // Initialize sensors
    dist_x.init();
    dist_y.init();
    limit.init();
    temp.init();
}

void loop() {
    // Read ambient temp
    double ambient_temp = temp.temperature();

    // Calibrate distance sensor
    DistanceSensor::calibrate(ambient_temp);

    // Print distance measurement
    Serial.print(ambient_temp);
    Serial.print(" ");
    Serial.print(dist_x.distance());
    Serial.print(" ");
    Serial.println(dist_y.distance());
}