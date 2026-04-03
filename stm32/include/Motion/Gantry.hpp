#pragma once

#include "Types/Point2.hpp"
#include "Motion/Motor.hpp"
#include "Types/PinDef.hpp"
#include "Types/VelocityProfile.hpp"

#include <Arduino.h>

/*
    Motor pairs and their motion:

    | LEFT    RIGHT   RESULT |
    |------------------------|
    | CW      CW      LEFT   |
    | CW      CCW     DOWN   |
    | CCW     CW      UP     |
    | CCW     CCW     RIGHT  |
*/

class Gantry {
    private:
        static const double _DRIVE_PULLEY_RADIUS;
        static const double _STEP_CONVERSION_CONST;

        // Motors
        static Motor _left;
        static Motor _right;

        // Position tracking
        static Point2<double> _position;            // stores the current position of the mallet

        // Brezenham's step tracking
        static Point2<double> _current_target;      // set to the last position the gantry was instructed to move the mallet to
        static Point2<int> _total_steps_to_target;  // stores the steps needed and direction for left and right motors
        static double _current_rpm;
        static uint16_t _current_period_us;
        static int _total_steps_larger;             // stores absolute value of the larger amount of steps the left or right motor needs to take
        static int _step_counter;                   // count up to _total_steps_larger each time complete a step
        static int _accel_steps;
        static int _decel_steps;
        
        // Brezenham's error tracking
        static Point2<int> _d;
        static int _err;

        // Velocity info
        static VelocityProfile _profile;

        // Timers
        static HardwareTimer* _step_period_timer;
        static HardwareTimer* _step_intermission_timer;

        /**
         * @brief 
         */
        static double _mapDouble(double x, double in_min, double in_max, double out_min, double out_max);

        /**
         * @brief Takes rpm returns unit [microseconds / microstep]
         */
        static uint16_t _calculateStepPeriod(double rpm);

         /**
         * @brief Input a target point and reuturns the required steps from the left
         *          and right motors to reach that point.
         */
        static Point2<int> _calculateSteps(const Point2<double>& target);

        /**
         * @brief Called as callback ISR from _start_motors.
         *          Steps one or both motors once as decided by Bresenham's line algorithm.
         *          Updates the current assumed position of the gantry controlled mallet.
         *          Updates the current step period in accordance with the velocity profile.
         */
        static void _stepMotion();

        /**
         * @brief Should be triggered as interrupt by timer 2 microseconds after
         *          pulling step pins high in _stepMotion function.
         *          2 microseconds is the pulse width specified by our motor driver chip.
         *          After pulling pins low will restart timer for _stepMotion
         *          with overflow set to current step period if there are more steps to complete. 
         */
        static void _stepIntermission();

    public:
        static const double DIST_TOLERANCE_LOW;
        static const double DIST_TOLERANCE_HIGH;

        /**
         * @brief Initializes the gantry, its motors, and its pins
         */
        static void init();

        /**
         * @brief Set position of gantry.
         */
        static void setPosition(const Point2<double>& pos);

        /**
         * @brief Get position of gantry.
         */
        static Point2<double> getPosition() { return _position; }

        /**
         * @brief Set how the velocity should change over the course of a movement.
         *        The parameters are:
         *        starting rpm minRPM, maximum rpm maxRPM, (maximum 800 rpm)
         *        percent of the movement to accelerate accelPercent (value from 0 to 1)
         *        percent of the movement to decelerate decelPercent (value from 0 to 1).
         */
        static void setVelocityProfile(const VelocityProfile& profile);

        /**
         * @brief Returns the velocity profile
         * 
         * @return The velocity profile
         */
        static VelocityProfile getVelocityProfile() { return _profile; }

        /**
         * @brief Returns current step counter value
         * 
         * @return step counter value
         */
        static int getStepCount() { return _step_counter; }

        /**
         * @brief Returns total steps need to go for movement
         * 
         * @return left or right motor total steps needed, whichever is larger
         */
        static int getTotalSteps() { return _total_steps_larger; }

        /**
         * @brief Call first when receive movement command to a point. Initilizes the proper
         *          variables for a straight line movement using Bresenham's line algorithm.
         *          Calculates steps required and direction for left and right motor. Uses 
         *          set velocity profile from prior call to setVelocityProfile().
         */
        static void initMotion(const Point2<double>& target);

        /**
         * @brief Resumes _stepMotion hardware timer if needed.
         *          Call after initMotion().
         */
        static void startMotion();

        /**
         * @brief Can be called to pause stepping motion e.g
         *          when a limit switch is triggered.
         */
        static void pauseMotion();

        /**
         * @brief Laptop can request to run this homing routine.
         *          This routine slowly backs mallet into the left
         *          and bottom limit switches to get a good reading on
         *          its position. The distance sensors are also tested
         *          when the mallet triggers both limit switches to deem
         *          if the distance sensors are reporting good readings.
         */
        static void home();
};
