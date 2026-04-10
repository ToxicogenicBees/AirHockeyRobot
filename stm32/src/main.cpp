#include "Sensors/TemperatureSensor.hpp"
#include "Sensors/DistanceSensor.hpp"
#include "Sensors/LimitSwitch.hpp"
#include "Sensors/EstopTriggered.hpp"
#include "Motion/Gantry.hpp"
#include "PinOut.hpp"
#include "Comms/SerialLink.hpp"
#include "Types/Point2.hpp"
#include "Types/VelocityProfile.hpp"

#include <Arduino.h>
#include <algorithm>

// Distances buffer, make odd so can easily get median of samples
constexpr uint8_t BUFFER_SIZE = 11;

Point2<double> distance_buffer[BUFFER_SIZE];
size_t distance_buffer_index = 0;

// Sensor definitions
DistanceSensor dist_x(dist_x_trig, dist_x_echo);
DistanceSensor dist_y(dist_y_trig, dist_y_echo);
TemperatureSensor temp(temp_read);

uint8_t pressed_switches = 0; // stores any limit switches pressed
bool use_switches = true;
LimitSwitch limit_l(lim_l);
LimitSwitch limit_r(lim_r);
LimitSwitch limit_b(lim_b);
LimitSwitch limit_t(lim_t);

EstopTriggered estop(estop_trig);

uint8_t readLimitSwitches() {
    uint8_t pressed_switches = 0;

    auto process_pressed = [&](LimitSwitch& limit, uint8_t flag, const Point2<double>& new_pos) {
        if (limit.pressed()) {
            Gantry::pauseMotion();
            pressed_switches |= flag;

            Gantry::setPosition(new_pos);

            // tell gantry to back up a bit away from limit switch towards middle of table
            Gantry::setVelocityProfile({0, 0, 100, 100});
            Gantry::initMotion(new_pos - 25.0 * (new_pos - Constants::Mallet::HOME*25.4).normal());

            Gantry::setPosition({
                dist_x.distanceBurstMedian(5),
                dist_y.distanceBurstMedian(5)
            });

            delayMicroseconds(1000);
        }
    };

    process_pressed(limit_l, Constants::LimitSwitch::LEFT_PRESSED, {
        25.4 * Constants::Mallet::LIMIT_BL.x, Gantry::getPosition().y
    });
    process_pressed(limit_r, Constants::LimitSwitch::RIGHT_PRESSED, {
        25.4 * Constants::Mallet::LIMIT_TR.x, Gantry::getPosition().y
    });
    process_pressed(limit_b, Constants::LimitSwitch::BOTTOM_PRESSED, {
        Gantry::getPosition().x, 25.4 * Constants::Mallet::LIMIT_BL.y
    });
    process_pressed(limit_t, Constants::LimitSwitch::TOP_PRESSED, {
        Gantry::getPosition().x, 25.4 * Constants::Mallet::LIMIT_TR.y
    });

    return pressed_switches;
}

void setup() {
    // Create serial handlers
    SerialLink::registerHandler(Action::VelocityProfile, [](Packet& packet) {
        const auto profile = packet.read<VelocityProfile>();
        Gantry::setVelocityProfile(profile);
    });
    SerialLink::registerHandler(Action::MalletPosition, [](Packet& packet) {
        const auto target = packet.read<Point2<double>>();
        Gantry::initMotion(target);
    });
    SerialLink::registerHandler(Action::DistanceSensorRead, [](Packet& packet) {
        // read distance sensors
        // DistanceSensor::calibrate(temp.temperature());

        Point2<double> median_distance{dist_x.distanceBurstMedian(10), dist_y.distanceBurstMedian(10)};

        // If delta distance lower than DIST_TOLERANCE_LOW than assume ok and don't edit Gantry position.
        // If delta distance greater than DIST_TOLERANCE_LOW away but smaller than 
        // DIST_TOLERANCE_HIGH from the current assumed mallet position, then set 
        // mallet position to the weighted average between sensed position and assumed. 
        // Otherwise distance sensor readings are bad, or the mallet
        // assumed position is way off, so request a mallet homing routine
        // using limit switches.
        double delta = (median_distance - Gantry::getPosition()).magnitude();

        if (delta > Gantry::DIST_TOLERANCE_LOW) {
            if (delta < Gantry::DIST_TOLERANCE_HIGH) {
                // set to weighted average between assumed position and sensed position 
                Gantry::setPosition(0.8*median_distance +  0.2*Gantry::getPosition());
                Packet packet(Action::DistanceSensorRead);
                SerialLink::buffer(packet); 
            } else {
                Packet packet(Action::MalletHome);
                SerialLink::buffer(packet); 
            }
        }
    });
    SerialLink::registerHandler(Action::MalletHome, [](Packet& packet) {
        // run mallet homing routine using limit switches
        Gantry::home();
    });
    SerialLink::registerHandler(Action::Ping, [](Packet& packet) {
        SerialLink::buffer({Action::Ping});
    });

    // Initialize Serial output
    SerialLink::init();

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
    Gantry::setVelocityProfile(VelocityProfile(0, 0, 100, 100));
    // DistanceSensor::calibrate(temp.temperature());; 
    Gantry::setPosition({dist_x.distanceBurstMedian(5), dist_y.distanceBurstMedian(5)}); 

    // Disable switches if they're unplugged
    if (readLimitSwitches())
        use_switches = false;
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

    // Serial.println(dist_y.distance());

    // Check if any limit switch is pressed for more than 5 loops (to filter any noise)
    // If so, stop movement and let laptop know

    if (use_switches) {
        auto pressed_switches = readLimitSwitches();
        if (pressed_switches) {
            Packet packet(Action::LimitSwitches);
            packet << pressed_switches;
            SerialLink::buffer(packet);  
        }
    }
    
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
