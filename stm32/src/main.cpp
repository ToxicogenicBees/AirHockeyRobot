#include "Sensors/DistanceSensor.h"
#include "Sensors/LimitSwitch.h"
#include "Motor/Motor.h"
#include "Pin.h"

#include <Arduino.h>

// Pin definitions
Pin DISTANCE_ECHO(D4, INPUT);
Pin DISTANCE_TRIG(D2, OUTPUT);
// Pin LIMIT;

// // Sensor definitions
DistanceSensor dist(DISTANCE_TRIG, DISTANCE_ECHO);
// LimitSwitch limit(LIMIT);

void setup() {
    // Initialize Serial output
    Serial.begin(115200);

    // // Initialize sensors
    dist.init();
    // limit.init();
}

void loop() {
    Serial.println(dist.distance());
}