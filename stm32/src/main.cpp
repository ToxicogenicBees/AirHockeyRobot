#include "Sensors/DistanceSensor.h"
#include "Sensors/LimitSwitch.h"
#include "Motion/Motor.h"
#include "PinOut.h"

#include <Arduino.h>

// // Sensor definitions
DistanceSensor dist(dist_trig, dist_echo);
LimitSwitch limit(lim);

void setup() {
    // Initialize Serial output
    SerialLink::init();

    // // Initialize sensors
    dist.init();
    limit.init();
}

void loop() {
    // Prints the distance read by the distance sensor
    Serial.println(dist.distance());
    Serial.println(limit.pressed());
}
