#pragma once

#include <iostream>

/*
    Link to stepper motor's torque-speed curve:
    https://www.omc-stepperonline.com/nema-23-bipolar-3nm-425oz-in-4-2a-57x57x114mm-4-wires-stepper-motor-cnc-23hs45-4204s

    Estimated sample points:
        T(1.01207590569) = 285.074626866
        T(3.01322599195) = 282.089552239
        T(7.01552616446) = 282.089552239
        T(10.0057504313) = 240.298507463
        T(12.9959746981) = 200.995024876
        T(16.9982748706) = 161.691542289
        T(20.0115008626) = 128.855721393
        T(23.0017251294) = 116.915422886
        T(27.0040253019) = 96.0199004975
        T(33.0074755607) = 77.6119402985
        T(37.0097757332) = 62.6865671642
        T(40.0000000000) = 41.7910447761
*/

class TorqueSpeed {
    private:
        static const int _N = 11;

        static double _torques[_N + 1], _speeds[_N + 1];    // Torque-speed pairs from curve
        static double _b[_N], _c[_N], _d[_N];               // Cubic spline constants

    public:
        /**
         * @brief Returns the requred torque in [N*m] to drive the stepper motor at a desired RPM
         * 
         * @param rpm   The motor's RPM
         * 
         * @return The required torque in [N*m]
         */
        static double getTorque(double rpm);
};

double TorqueSpeed::_torques[_N + 1] = { 285.074626866, 282.089552239, 282.089552239, 240.298507463, 200.995024876, 161.691542289, 128.855721393, 116.915422886, 96.0199004975, 77.6119402985, 62.6865671642, 41.7910447761 };
double TorqueSpeed::_speeds[_N + 1] = { 1.01207590569, 3.01322599195, 7.01552616446, 10.0057504313, 12.9959746981, 16.9982748706, 20.0115008626, 23.0017251294, 27.0040253019, 33.0074755607, 37.0097757332, 40.0000000000 };

double TorqueSpeed::_b[_N] = { -2.552138571324805, 0.629238542353168, -7.621231312046389, -15.767280682924650, -10.669290043272882, -11.235504783664551, -7.450389367332408, -3.583924742965864, -4.962713096685230, -2.503190508981127, -5.667613043027798 };
double TorqueSpeed::_c[_N] = { -0.000000000000001, 1.589774368010413, -3.651206419276413, 0.926979523918217, 0.777906191930582, -0.919378524336372, 2.175545644353749, -0.882510647850208, 0.538011661194212, -0.128326816367044, -0.662324158737116 };
double TorqueSpeed::_d[_N] = { 0.264810117429620, -0.436497394113912, 0.510350343279631, -0.016617854123605, -0.141359105440506, 0.342371064202400, -0.340894864883222, 0.118308826409781, -0.036997529133880, -0.044474203945621, 0.073832161028946 };

double TorqueSpeed::getTorque(double rpm) {
    // Convert speed from KPPS to RPM
    double speed = rpm / 30;

    // Saturate speed if it goes beyond the inputs range
    if (speed > _speeds[_N]) speed = _speeds[_N];
    if (speed < _speeds[0])  speed = _speeds[0];
    
    // Determine which spline to use
    int i = 0;
    while (i < _N && speed > _speeds[i + 1])
        i++;
        
    // Calculate torque (N * cm)
    double ds = speed - _speeds[i];
    double n_cm = ds * ds * ds * _d[i]
        + ds * ds * _c[i]
        + ds * _b[i]
        + _torques[i];

    // Return torque in N * m
    return n_cm / 100.0;
}