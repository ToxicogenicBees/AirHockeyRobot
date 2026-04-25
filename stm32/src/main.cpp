#include "Sensors/TemperatureSensor.hpp"
#include "Sensors/DistanceSensor.hpp"
#include "Sensors/LimitSwitch.hpp"
#include "Sensors/EmergencyStop.hpp"
#include "Motion/Gantry.hpp"
#include "PinOut.hpp"
#include "Comms/SerialLink.hpp"
#include "Types/Point2.hpp"
#include "Types/VelocityProfile.hpp"

#include <Arduino.h>
#include <algorithm>

// Sensor definitions
DistanceSensor dist_x(dist_x_trig, dist_x_echo);
DistanceSensor dist_y(dist_y_trig, dist_y_echo);
TemperatureSensor temp(temp_read);

LimitSwitch limit_l(lim_l);
LimitSwitch limit_r(lim_r);
LimitSwitch limit_b(lim_b);
LimitSwitch limit_t(lim_t);

EmergencyStop estop(estop_trig);
bool gantry_enabled = true;

// Limit switch usage flag
bool use_switches = true;

// Read the limit switches and store them in a flag byte
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
            delayMicroseconds(1000);
        }
    };

    process_pressed(limit_l, Constants::LimitSwitch::LEFT_PRESSED, {
        25.4 * (Constants::Mallet::LIMIT_BL.x - Constants::Mallet::SENSOR_OFFSET.x), Gantry::getPosition().y
    });
    process_pressed(limit_r, Constants::LimitSwitch::RIGHT_PRESSED, {
        25.4 * (Constants::Mallet::LIMIT_TR.x - Constants::Mallet::SENSOR_OFFSET.x), Gantry::getPosition().y
    });
    process_pressed(limit_b, Constants::LimitSwitch::BOTTOM_PRESSED, {
        Gantry::getPosition().x, 25.4 * (Constants::Mallet::LIMIT_BL.y - Constants::Mallet::SENSOR_OFFSET.y)
    });
    process_pressed(limit_t, Constants::LimitSwitch::TOP_PRESSED, {
        Gantry::getPosition().x, 25.4 * (Constants::Mallet::LIMIT_TR.y - Constants::Mallet::SENSOR_OFFSET.y)
    });

    return pressed_switches;
}

// Function called on microcontroller startup
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
            }
        }
    });
    SerialLink::registerHandler(Action::Ping, [](Packet& packet) {
        SerialLink::buffer({Action::Ping});
    });

    // Initialize Serial output
    SerialLink::init();
    
    // Initialize ADC converter precision
    analogReadResolution(12);
    
    // Initialize sensors
    dist_x.init();
    dist_y.init();
    limit_l.init();
    limit_r.init();
    limit_b.init();
    limit_t.init();
    estop.init();
    temp.init();
    delay(100);
    
    // Calibrate distance sensor
    Gantry::init();   
    Gantry::setVelocityProfile(VelocityProfile(0, 0, 100, 100));
    Gantry::setPosition({dist_x.distanceBurstMedian(5), dist_y.distanceBurstMedian(5)}); 

    // Disable switches if they're unplugged
    if (readLimitSwitches())
        use_switches = false;    
}

// Function called in a loop after microcontroller startup
void loop() {
    // Process serial data
    SerialLink::process();

    // Update estop state
    // If enabled, ignore any further processing this update
    estop.update();
    if (estop.enabled()) {
        gantry_enabled = false;
        return;
    }

    // Re-enable the gantry if it was deactivated
    if (!gantry_enabled) {
        Gantry::initMotors();
        Gantry::setPosition({dist_x.distanceBurstMedian(5), dist_y.distanceBurstMedian(5)}); 
        gantry_enabled = true;
    }

    /*
        Enable to calibrade distance sensors with a temperature sensor
    
        // Calibrate distance sensor
        DistanceSensor::calibrate(temp.temperature());
        Gantry::setPosition({dist_x.distance(), dist_y.distance(),});
    */
    
    // Transmit mallet position
    Packet packet(Action::MalletPosition);
    packet << Gantry::getPosition();
    SerialLink::buffer(packet);

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
}
