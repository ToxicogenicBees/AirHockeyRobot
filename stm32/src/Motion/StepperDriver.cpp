#include "Motion/StepperDriver.h"

void StepperDriver::_sendData(uint8_t reg, uint16_t data) {

}

StepperDriver::StepperDriver() {
    // Initialize control register values
    setDirection();
    setDTime();
    setEnable();
    setMode();
    setRStep();
    setISGain();
    setStall();
}

void StepperDriver::sendControl() {
    uint16_t ctrl_data = 0x00;

    ctrl_data |= (_dtime << 11);
    ctrl_data |= (_isgain << 9);
    ctrl_data |= (_exstall << 8);
    ctrl_data |= (_mode << 4);
    ctrl_data |= (_rstep << 3);
    ctrl_data |= (_rdir << 2);
    ctrl_data |= (_enbl << 1);
    ctrl_data |= RW::WRITE;

    _sendData(RegAddress::CONTROL, ctrl_data);
}