#pragma once

#include "Types/PinDef.hpp"

/**********
  Gantry  
**********/

// SPI Pins
extern PinDef miso;
extern PinDef mosi;

// Left motor pins
extern PinDef motor_l_step;
extern PinDef motor_l_dir;
extern PinDef motor_l_scs;
extern PinDef motor_l_fault;

// Right motor pins
extern PinDef motor_r_step;
extern PinDef motor_r_dir;
extern PinDef motor_r_scs;
extern PinDef motor_r_fault;

// Motor pins common to both motors
extern PinDef motor_sleep;
extern PinDef motor_enable;

/**********
  Sensors  
**********/

// Distance sensor x-axis
extern PinDef dist_x_trig;
extern PinDef dist_x_echo;

// Distance sensor y-axis
extern PinDef dist_y_trig;
extern PinDef dist_y_echo;

// Limit Switch
extern PinDef lim_r;
extern PinDef lim_l;
extern PinDef lim_t;
extern PinDef lim_b;

// Temperature sensor
extern PinDef temp_read;

// Estop
extern PinDef estop_trig;
