#include "Sensors/DistanceSensor.h"
#include "Sensors/LimitSwitch.h"
#include "Motion/Motor.h"
#include "Types/PinDef.h"

#include <Arduino.h>

// Pin definitions
PinDef DISTANCE_ECHO(D4, INPUT);
PinDef DISTANCE_TRIG(D2, OUTPUT);
PinDef LIMIT(D1, INPUT);

// // Sensor definitions
DistanceSensor dist(DISTANCE_TRIG, DISTANCE_ECHO);
LimitSwitch limit(LIMIT);

void setup() {
    // Initialize Serial output
    Serial.begin(115200);

    // // Initialize sensors
    dist.init();
    limit.init();
}

void loop() {
    // Prints the distance read by the distance sensor
    Serial.println(dist.distance());
}