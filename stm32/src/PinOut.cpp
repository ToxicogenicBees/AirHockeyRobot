#include "PinOut.h"

PinDef miso(D12, OUTPUT);
PinDef mosi(D11, OUTPUT);

PinDef motor_l_step(A2, OUTPUT);
PinDef motor_l_dir(A3, OUTPUT);
PinDef motor_l_scs(A1, OUTPUT);

PinDef motor_r_step(D9, OUTPUT);
PinDef motor_r_dir(D8, OUTPUT);
PinDef motor_r_scs(D10, OUTPUT);

PinDef dist_x_trig(A5, OUTPUT);
PinDef dist_x_echo(A4, INPUT);
PinDef dist_y_trig(D2, OUTPUT);
PinDef dist_y_echo(D3, INPUT);

PinDef lim_b(D4, INPUT_PULLUP);
PinDef lim_t(D5, INPUT_PULLUP);
PinDef lim_l(D6, INPUT_PULLUP);
PinDef lim_r(D7, INPUT_PULLUP);

PinDef temp_read(A0, INPUT_ANALOG);