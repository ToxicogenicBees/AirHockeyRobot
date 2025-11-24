#include "Sensors/TemperatureSensor.h"
#include "Sensors/DistanceSensor.h"
#include "Sensors/LimitSwitch.h"
#include "Comms/SerialLink.h"
#include "Motion/Motor.h"
#include "PinOut.h"
#include "Types/Point2.hpp"

#include <Arduino.h>

// Sensor definitions
DistanceSensor dist_x(dist_x_trig, dist_x_echo);
DistanceSensor dist_y(dist_y_trig, dist_y_echo);
TemperatureSensor temp(temp_read);
LimitSwitch limitL(limL);
LimitSwitch limitR(limR);
LimitSwitch limitB(limB);
LimitSwitch limitT(limT);

void setup() {
    // Initialize Serial output
    SerialLink::init();

    // Initialize ADC converter precision
    analogReadResolution(12);

    // Initialize sensors
    dist_x.init();
    dist_y.init();
    limitL.init();
    limitR.init();
    limitB.init();
    limitT.init();
    temp.init();
}

void loop() {
    Packet packet(Action::MALLET_POSITION);

    // Read ambient temp
    double ambient_temp = temp.temperature();

    // Calibrate distance sensor
    DistanceSensor::calibrate(ambient_temp);
    double x = dist_x.distance();
    double y = dist_y.distance();

    // Print distance measurement
    // Serial.print("Temp: ");
    // Serial.print(ambient_temp);
    // Serial.print(" ");
    // Serial.print("Dist X: ");
    // Serial.print(x);
    // Serial.print(" ");
    // Serial.print("Dist Y: ");
    // Serial.print(y);
    // Serial.print(" ");
    // Serial.print("LimL: ");
    // Serial.print(limitL.pressed());
    // Serial.print(" ");
    // Serial.print("LimR: ");
    // Serial.print(limitR.pressed());
    // Serial.print(" ");
    // Serial.print("LimB: ");
    // Serial.print(limitB.pressed());
    // Serial.print(" ");
    // Serial.print("LimT: ");
    // Serial.println(limitT.pressed());
    

    // Point2<double> position(x, y);

    // packet << position;

    packet = SerialLink::read();    
    SerialLink::send(packet);

    // delay(2000);
}