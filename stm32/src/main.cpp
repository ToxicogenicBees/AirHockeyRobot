#include "Sensors/TemperatureSensor.h"
#include "Sensors/DistanceSensor.h"
#include "Sensors/LimitSwitch.h"
#include "Motion/Gantry.h"
#include "PinOut.h"
#include "Comms/SerialLink.hpp"
#include "Types/Point2.hpp"
#include "Types/VelocityProfile.hpp"

#include <Arduino.h>

// Distances buffer
constexpr uint8_t BUFFER_SIZE = 10;

Point2<double> distance_buffer[BUFFER_SIZE];
size_t distance_buffer_index = 0;

// Sensor definitions
DistanceSensor dist_x(dist_x_trig, dist_x_echo);
DistanceSensor dist_y(dist_y_trig, dist_y_echo);
TemperatureSensor temp(temp_read);
LimitSwitch limit_l(lim_l);
LimitSwitch limit_r(lim_r);
LimitSwitch limit_b(lim_b);
LimitSwitch limit_t(lim_t);

void HANDLE_PACKET(Packet& packet) {
    Action action = packet.action();
    packet.resetRead();

    switch(action) {
        case Action::VelocityProfile: {
            const auto profile = packet.read<VelocityProfile>();
            Gantry::setVelocityProfile(profile);
            break;
        }

        case Action::MalletPosition: {
            const auto target = packet.read<Point2<double>>();
            Gantry::initMotion(target);
            Gantry::startMotion();
            break;
        }

        case Action::DistanceSensorRead: {
            // read distance sensors
            DistanceSensor::calibrate(temp.temperature());

            // Calculate average distance
            Point2<double> avg_pos;
            for (int i = 0; i < BUFFER_SIZE; i++)
                avg_pos += {dist_x.distance(), dist_y.distance()}; 
            avg_pos /= BUFFER_SIZE;

            // If delta distance lower than DIST_TOLERANCE_LOW than assume ok and don't edit Gantry position.
            // If delta distance greater than DIST_TOLERANCE_LOW away but smaller than 
            // DIST_TOLERANCE_HIGH from the current assumed mallet position, then set 
            // mallet position to the weighted average between sensed position and assumed. 
            // Otherwise distance sensor readings are bad, or the mallet
            // assumed position is way off, so request a mallet homing routine
            // using limit switches.
            Point2<double> delta = {abs(avg_pos.x - Gantry::getPosition().x), abs(avg_pos.y - Gantry::getPosition().y)};

            if (delta.x > Gantry::DIST_TOLERANCE_LOW || delta.y > Gantry::DIST_TOLERANCE_LOW) {
                if (delta.x < Gantry::DIST_TOLERANCE_HIGH && delta.y < Gantry::DIST_TOLERANCE_HIGH) {
                    // set to weighted average between assumed position and sensed position 
                    Gantry::setPosition(0.8*avg_pos +  0.2*Gantry::getPosition());
                } else {
                    Packet packet(Action::MalletHome);
                    SerialLink::buffer(packet); 
                }
            }
            break;
        }

        case Action::MalletHome: {
            // run mallet homing routine using limit switches
            Gantry::home();
            break;
        }
    }
}

void setup() {
    // Initialize Serial output
    SerialLink::init(HANDLE_PACKET);

    // Serial.println(SystemCoreClock);  // gets the current core clock speed, this reported 180MHz
    
    // Initialize ADC converter precision
    analogReadResolution(12);
    
    // Initialize sensors
    dist_x.init();
    dist_y.init();
    limit_l.init();
    limit_r.init();
    limit_b.init();
    limit_t.init();
    temp.init();

    delay(100);
    
    // Calibrate distance sensor
    Gantry::init();   
    // DistanceSensor::calibrate(temp.temperature());
    // Gantry::setPosition({dist_x.distance(), dist_y.distance(),}); 
    Gantry::setPosition({250, 250}); 
}

void loop() {
    // // Calibrate distance sensor
    // DistanceSensor::calibrate(temp.temperature());
    // Gantry::setPosition({dist_x.distance(), dist_y.distance(),});

    // Process serial data
    SerialLink::process();

    Packet packet(Action::MalletPosition);
    packet << Gantry::getPosition();
    SerialLink::buffer(packet);  
    
    // // Read distance
    // for (size_t i = 0; i < BUFFER_SIZE; ++i) {
    //     //distance_buffer_index = (distance_buffer_index + 1) % BUFFER_SIZE;
    //     distance_buffer[i] = {
    //         dist_x.distance(),
    //         dist_y.distance()
    //     };
    // }
    
    // // Calculate average distance
    // Point2<double> avg_pos;
    // for (int i = 0; i < BUFFER_SIZE; i++)
    //     avg_pos += distance_buffer[i]; 
    // avg_pos /= BUFFER_SIZE;
    
    // // Buffer mallet position for the laptop
    // Packet packet(Action::MalletPosition);
    // packet << avg_pos;
    // SerialLink::buffer(packet);  
}
