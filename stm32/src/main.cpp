#include "Sensors/TemperatureSensor.h"
#include "Sensors/DistanceSensor.h"
#include "Sensors/LimitSwitch.h"
#include "Motion/Gantry.h"
#include "PinOut.h"
#include "Comms/SerialLink.hpp"
#include "Types/Point2.hpp"

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
            const double min_rpm = packet.read<uint8_t>() / 127.0;
            const double max_rpm = packet.read<uint8_t>() / 127.0;
            const double accel_percent = packet.read<uint8_t>() / 127.0;
            const double decel_percent = packet.read<uint8_t>() / 127.0;

            Gantry::setVelocityProfile(min_rpm, max_rpm, accel_percent, decel_percent);
        }

        case Action::MalletPosition: {
            const auto target = packet.read<Point2<double>>();
            Gantry::goToPointInStraightLine(target);
        }
    }
}

void setup() {
    // Initialize Serial output
    SerialLink::init(HANDLE_PACKET);

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
}

void loop() {
    // Process serial data
    SerialLink::process();

    // Calibrate distance sensor
    DistanceSensor::calibrate(temp.temperature());
    
    // Read distance
    distance_buffer_index = (distance_buffer_index + 1) % BUFFER_SIZE;
    distance_buffer[distance_buffer_index] = {
        dist_x.distance(),
        dist_y.distance()
    };
    
    // Calculate average distance
    Point2<double> avg_pos;
    for (int i = 0; i < BUFFER_SIZE; i++)
        avg_pos += distance_buffer[i]; 
    avg_pos /= BUFFER_SIZE;
    
    // Buffer mallet position for the laptop
    Packet packet(Action::MalletPosition);
    packet << avg_pos;
    SerialLink::buffer(packet);  
}
