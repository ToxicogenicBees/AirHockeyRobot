#ifndef GANTRY_H
#define GANTRY_H

#include "Types/Point2.hpp"
#include "Motion/Motor.h"
#include "Types/PinDef.h"

#include <Arduino.h>

class Gantry {
    private:
        static const float _DRIVE_PULLEY_RADIUS;
        static const float _STEP_CONVERSION_CONST;

         /*
            Motor pairs and their motion:

            | LEFT    RIGHT   RESULT |
            |------------------------|
            | CW      CW      LEFT   |
            | CW      CCW     DOWN   |
            | CCW     CW      UP     |
            | CCW     CCW     RIGHT  |
        */
        static Motor _left;
        static Motor _right;

        static Point2<double> _position;  // stores the current position of the mallet

        static Point2<double> _current_target;  // set to the last position the gantry was instructed to move the mallet to
        static Point2<int> _total_steps_to_target;  // stores the steps needed and direction for left and right motors
        static int _total_steps_larger;  // stores absolute value of the larger amount of steps the left or right motor needs to take
        static int _step_counter;   // count up to _total_steps_larger each time complete a step
        static int _accel_steps;
        static int _decel_steps;
        static double _current_rpm;
        static Point2<int> _d;
        static int _err;
        static uint16_t _current_period_us;

        static double _accel_percent;
        static double _decel_percent;
        static double _min_rpm;
        static double _max_rpm;

        static HardwareTimer *_increment_straight_line_movement_timer;
        static HardwareTimer *_pull_down_motor_step_pins_timer;

        /**
         * @brief 
         */
        static float _mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

        /**
         * @brief Takes rpm returns unit [microseconds / microstep]
         */
        static uint16_t _calculateStepPeriod(double rpm);

         /**
         * @brief Input a target point and reuturns the required steps from the left
         *          and right motors to reach that point.
         */
        static Point2<int> _calculateMotorSteps(const Point2<double>& target);

        /**
         * @brief Called as callback ISR from _increment_straight_line_movement_timer.
         *          Steps one or both motors as decided by Bresenham's line algorithm.
         *          Updates the current assumed position of the gantry controlled mallet.
         *          Updates the current step period in accordance with the velocity profile.
         */
        static void _incrementStraightLineMovement();

        /**
         * @brief Should be triggered as interrupt by timer 2 microseconds after
         *          pulling step pins high in incrementStraightLineMovement function.
         *          2 microseconds is the pulse width specified by our motor driver chip.
         *          After pulling pins low will restart timer for incrementStraightLineMovement
         *          with overflow set to current step period if there are more steps to complete. 
         */
        static void _pullDownMotorStepPinsAndRestartIncrementTimer();

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
         * @brief Set position of gantry.
         */
        static Point2<double> getPosition() {return _position;}

        /**
         * @brief Returns the acceleration percent
         * 
         * @return The acceleration percent
         */
        static double getAccelPercent() { return _accel_percent; }

        /**
         * @brief Returns the deceleration percent
         * 
         * @return The deceleration percent
         */
        static double getDecelPercent() { return _decel_percent; }

        /**
         * @brief Returns the min RPM
         * 
         * @return The min RPM
         */
        static double getMinRPM() { return _min_rpm; }

        /**
         * @brief Returns the max RPM
         * 
         * @return The max RPM
         */
        static double getMaxRPM() { return _max_rpm; }

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
         * @brief Set how the velocity should change over the course of a movement.
         *        The parameters are:
         *        starting rpm minRPM, maximum rpm maxRPM, (maximum 800 rpm)
         *        percent of the movement to accelerate accelPercent (value from 0 to 1)
         *        percent of the movement to decelerate decelPercent (value from 0 to 1).
         */
        static void setVelocityProfile(
            double min_rpm, 
            double max_rpm, 
            double accel_percent, 
            double decel_percent
        );

        /**
         * @brief Call first when receive movement command to a point. Initilizes the proper
         *          variables for a straight line movement using Bresenham's line algorithm.
         *          Calculates steps required and direction for left and right motor. Uses 
         *          set velocity profile from prior call to setVelocityProfile().
         */
        static void setUpStraightLineMovement(const Point2<double>& target);

        /**
         * @brief Resumes _incrementStraightLineMovement hardware timer if needed.
         *          Call after setUpStraightLineMovement().
         */
        static void startOrContiueStraightLineMovement();

        /**
         * @brief Laptop can request to run this homing routine.
         *          This routine slowly backs mallet into the left
         *          and bottom limit switches to get a good reading on
         *          its position. The distance sensors are also tested
         *          when the mallet triggers both limit switches to deem
         *          if the distance sensors are reporting good readings.
         */
        static void runHomingRoutine();
};

#endif
