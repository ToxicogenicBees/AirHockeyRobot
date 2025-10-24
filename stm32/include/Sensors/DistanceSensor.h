#pragma once

#include <stdint.h>
#include "Sensor.h"
#include "Pin.h"

class DistanceSensor : public Sensor {
    private:
        uint32_t _update_time;
        double _cur_dist;

        void _onInterrupt() override;

    public:
        DistanceSensor(Pin& trig, Pin& echo);

        void update();

        uint16_t distance() { return _cur_dist; }
};