#include "Sensors/TemperatureSensor.h"
#include "Sensors/DistanceSensor.h"
#include "Sensors/LimitSwitch.h"
#include "Comms/SerialLink.h"
#include "Motion/Motor.h"
#include "PinOut.h"

#include <Arduino.h>

// Sensor definitions
DistanceSensor dist(dist_trig, dist_echo);
TemperatureSensor temp(temp_read);
LimitSwitch limit(lim);

void setup() {
    // Initialize Serial output
    SerialLink::init();

    // Initialize sensors
    dist.init();
    limit.init();
    temp.init();
}

void loop() {
    // Prints the distance read by the distance sensor
    Serial.println(temp.temperature());
}