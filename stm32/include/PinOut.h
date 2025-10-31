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

// Distance sensor
extern PinDef dist_trig;
extern PinDef dist_echo;

// Limit Switch
extern PinDef lim;