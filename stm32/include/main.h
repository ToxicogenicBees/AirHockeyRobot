#pragma once

// Hardware abstraction layer
#include "stm32f4xx_hal.h"

// Custom classes
#include "Sensors/DistanceSensor.h"
#include "Sensors/LimitSwitch.h"
#include "Motor/Motor.h"
#include "Pin.h"

// Pin definitions
Pin DISTANCE_X_ECHO;
Pin DISTANCE_X_TRIG;
Pin DISTANCE_Y_ECHO;
Pin DISTANCE_Y_TRIG;

Pin LIMIT_X;
Pin LIMIT_Y;

// Sensor definitions
DistanceSensor x_dist(DISTANCE_X_TRIG, DISTANCE_X_ECHO);
DistanceSensor y_dist(DISTANCE_Y_TRIG, DISTANCE_Y_ECHO);

LimitSwitch x_limit(LIMIT_X);
LimitSwitch y_limit(LIMIT_Y);