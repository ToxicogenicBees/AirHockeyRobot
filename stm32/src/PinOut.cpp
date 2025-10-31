#include "PinOut.h"

PinDef miso(D12, OUTPUT);
PinDef mosi(D11, OUTPUT);
PinDef sclk;

PinDef motor_l_step;
PinDef motor_l_dir;
PinDef motor_l_scs;

PinDef motor_r_step;
PinDef motor_r_dir;
PinDef motor_r_scs;

PinDef dist_trig(D2, OUTPUT);
PinDef dist_echo(D4, INPUT);
PinDef lim(D7, INPUT_PULLDOWN);