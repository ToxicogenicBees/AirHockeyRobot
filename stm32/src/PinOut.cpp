#include "PinOut.h"

PinDef miso(D12, OUTPUT);
PinDef mosi(D11, OUTPUT);

PinDef motor_l_step(PB10, OUTPUT);
PinDef motor_l_dir(PB4, OUTPUT);
PinDef motor_l_scs(PB5, OUTPUT);
PinDef motor_l_fault(PB12, OUTPUT);

PinDef motor_r_step(PC7, OUTPUT);
PinDef motor_r_dir(PA9, OUTPUT);
PinDef motor_r_scs(PA8, OUTPUT);
PinDef motor_r_fault(PA10, OUTPUT);

PinDef motor_sleep(PB3, OUTPUT);
PinDef motor_enable(PB6, OUTPUT);

PinDef dist_x_trig(PA1, OUTPUT);
PinDef dist_x_echo(PA0, INPUT);
PinDef dist_y_trig(PB0, OUTPUT);
PinDef dist_y_echo(PA4, INPUT);

PinDef lim_b(PB7, INPUT_PULLUP);
PinDef lim_t(PC14, INPUT_PULLUP);
PinDef lim_l(PC15, INPUT_PULLUP);
PinDef lim_r(PC13, INPUT_PULLUP);

PinDef temp_read(PA12, INPUT_ANALOG);