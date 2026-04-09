#pragma once

#include "Types/Point2.hpp"

namespace Constants {
    namespace Comms {
        constexpr uint32_t BAUD_RATE = 115200;          // Baud rate for USB serial communication
        constexpr char COM_PORT[] = "\\\\.\\COM6";      // Desired COM port
        constexpr int64_t TIMEOUT = 5;                  // Timeout in seconds
    }
    
    namespace Table {
        const Point2<double> HUMAN_GOAL(13.25, 54.0);   // Center of human's goal (inches)
        const Point2<double> ROBOT_GOAL(13.25, 0.0);    // Center of robot's goal (inches)
        const Point2<double> SIZE(26.5, 54.0);          // Size of the table (inches)
        
        constexpr double GOAL_WIDTH = 9.0;              // Width of goals (inches)
        constexpr double COEF_FRIC = 0.05;              // Coefficient of friction for the surface of the table
        constexpr double COEF_REST = 1.00;              // Coefficient of friction of the walls of the table
    }

    namespace Camera {
        constexpr int DEVICE_ID = 0;                    // Camera device id
        constexpr int API_ID = 0;                       // Camera API id (cv::CAP_ANY)
    }

    namespace Mallet {
        const Point2<double> HOME = {13.25, 10.0};      // Homing location for the mallet
        const Point2<double> DODGE_LEFT = {5.0, 5.0};   // Left dodging location for the mallet
        const Point2<double> DODGE_RIGHT = {21.5, 5.0}; // Right dodging location for the mallet
        constexpr double SPEED = 80.4251969;            // Estimated mallet speed (inch/sec)
        constexpr double ACCEL = 17 * 39.37;            // Estimated mallet accelleration approx. linear
                                                        // from 250 to 800 RPM speed test (inch/sec^2)
        constexpr double RADIUS = 2.0;                  // Radius in inches
        constexpr double MAX_RPM = 1200;
        constexpr double MIN_RPM = 350;
        constexpr double INCHES_TO_ACCEL_TO_MAX_RPM = 7.48; // taken from the motor speed test data accelerating to 800RPM
        constexpr double MIN_ACCEL_INCHES = 1;   // lower bound of distance to accelerate to certain speed
        constexpr double DRIVE_PULLEY_RADIUS_MM = 28.8;
        const double MAX_SPEED_INCHES_PER_SECOND = MAX_RPM * 2*3.14/60 * (DRIVE_PULLEY_RADIUS_MM*sqrt(2)/2)/25.4;

        const Point2<double> LIMIT_BL                   // Bottom-left limit of mallet motion
            = {2.5, 2.75};
        const Point2<double> LIMIT_TR                   // Top-right limit of mallet motion
            = {Table::SIZE.x - LIMIT_BL.x, 21.0};
        const Point2<double> SENSOR_OFFSET              // Sensor offset
            = {1.5, 1.25};
    }
    
    namespace Puck {
        const Point2<double> HOME = {13.25, 50.0};      // Homing location for the puck (initialization)
        constexpr double SPEED = 137.795276;            // Estimated puck speed (inch/sec)
        constexpr double RADIUS = 1.25;                 // Radius in inches
    }

    namespace LimitSwitch {
        const uint8_t LEFT_PRESSED = 0x01;
        const uint8_t RIGHT_PRESSED = 0x02;
        const uint8_t BOTTOM_PRESSED = 0x04;
        const uint8_t TOP_PRESSED = 0x08;
    }
    
    // General
    constexpr int64_t SAMPLE_PERIOD = 1e6 / 100;        // Camera sample period (microseconds)
    constexpr size_t MAX_SAMPLE_POINTS = 50;            // Maximum number of sample points
    constexpr double FP_ERR = 1e-8;                     // Consistant FP error bias
}
