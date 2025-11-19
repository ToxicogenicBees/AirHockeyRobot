#pragma once

#include "Types/PinDef.h"

/**********
  Gantry  
**********/

// SPI Pins
extern PinDef miso;
extern PinDef mosi;
extern PinDef sclk;

// Left motor pins
extern PinDef motor_l_step;
extern PinDef motor_l_dir;
extern PinDef motor_l_scs;

// Right motor pins
extern PinDef motor_r_step;
extern PinDef motor_r_dir;
extern PinDef motor_r_scs;


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
extern PinDef limR;
extern PinDef limL;
extern PinDef limT;
extern PinDef limB;

// Temperature sensor
extern PinDef temp_read;