#pragma once

#include <stdint.h>

// Register constants
namespace RegAddress {
    constexpr uint8_t CONTROL = 0x00;
    constexpr uint8_t TORQUE = 0x01;
    constexpr uint8_t OFF = 0x02;
    constexpr uint8_t BLANK = 0x03;
    constexpr uint8_t DECAY = 0x04;
    constexpr uint8_t STALL = 0x05;
    constexpr uint8_t DRIVE = 0x06;
    constexpr uint8_t STATUS = 0x07;
};

// Read/Write constants
namespace RW {
    constexpr uint8_t READ = 0x00;
    constexpr uint8_t WRITE = 0x01;
}

// Control register constants
namespace Control {
    // Enabled state
    namespace Enable {
        constexpr uint8_t DISABLED = 0x00;
        constexpr uint8_t ENABLED = 0x01;
    }

    // Motor direction
    namespace Direction {
        constexpr uint8_t FWD = 0x00;
        constexpr uint8_t REV = 0x01;
    };

    // Step
    namespace RStep {
        constexpr uint8_t NO_ACTION = 0x00;
        constexpr uint8_t ADVANCE = 0x01;
    };

    // Step mode control
    namespace Mode {
        constexpr uint8_t STEP_1 = 0x00;
        constexpr uint8_t STEP_2 = 0x01;
        constexpr uint8_t STEP_4 = 0x02;
        constexpr uint8_t STEP_8 = 0x03;
        constexpr uint8_t STEP_16 = 0x04;
        constexpr uint8_t STEP_32 = 0x05;
        constexpr uint8_t STEP_64 = 0x06;
        constexpr uint8_t STEP_128 = 0x07;
        constexpr uint8_t STEP_256 = 0x08;
    };

    // Stall detection
    namespace Stall {
        constexpr uint8_t INT = 0x00;
        constexpr uint8_t EXT = 0x01;
    }

    // ISENSE amplifier gain
    namespace ISGain {
        constexpr uint8_t GAIN_5 = 0x00;
        constexpr uint8_t GAIN_10 = 0x01;
        constexpr uint8_t GAIN_20 = 0x02;
        constexpr uint8_t GAIN_40 = 0x03;
    };

    // Dead time
    namespace DTime {
        constexpr uint8_t DEAD_400_NS = 0x00;
        constexpr uint8_t DEAD_450_NS = 0x01;
        constexpr uint8_t DEAD_650_NS = 0x02;
        constexpr uint8_t DEAD_850_NS = 0x03;
    };
};

class StepperDriver {
    private:
        uint8_t _enbl, _rdir, _rstep, _mode, _exstall, _isgain, _dtime;

        void _sendData(uint8_t reg, uint16_t data);

    public:
        StepperDriver();

        void sendControl();

        void setDirection(uint8_t state = Control::Direction::FWD) { _rdir = state; }
        void setDTime(uint8_t state = Control::DTime::DEAD_850_NS) { _dtime = state; }
        void setEnable(uint8_t state = Control::Enable::DISABLED) { _enbl = state; }
        void setISGain(uint8_t state = Control::ISGain::GAIN_5) { _isgain = state; }
        void setMode(uint8_t state = Control::Mode::STEP_4) { _mode = state; }
        void setRStep(uint8_t state = Control::RStep::NO_ACTION) { _rstep = state; }
        void setStall(uint8_t state = Control::Stall::INT) { _exstall = state; }
};