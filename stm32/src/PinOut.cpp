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

PinDef dist_x_trig(A5, OUTPUT);
PinDef dist_x_echo(A4, INPUT);
PinDef dist_y_trig(D2, OUTPUT);
PinDef dist_y_echo(D3, INPUT);
PinDef limB(D4, INPUT_PULLUP);
PinDef limT(D5, INPUT_PULLUP);
PinDef limL(D6, INPUT_PULLUP);
PinDef limR(D7, INPUT_PULLUP);
PinDef temp_read(A0, INPUT_ANALOG);