#include <opencv2/opencv.hpp>
#include <iostream>

#include "Comms/SerialLink.hpp"
#include "Comms/Packet.hpp"
#include "Motion/Table.hpp"
#include "Constants.hpp"

MovingObject Table::_mallet(Constants::Mallet::RADIUS);
MovingObject Table::_puck(Constants::Puck::RADIUS);

std::unique_ptr<PuckTracker> Table::_tracker = nullptr;
std::unique_ptr<Routine> Table::_routine = nullptr;

void Table::init() {
    // Initialize serial comms
    SerialLink::registerHandler(Action::MalletPosition, [](Packet& packet) {
        // Convert offset millimeter position to inches
        const auto millimeters = packet.read<Point2<double>>();
        const auto inches = (millimeters / 25.4) + Constants::Mallet::SENSOR_OFFSET;

        // Move mallet
        _mallet.moveTo(inches);
    });
    SerialLink::registerHandler(Action::LimitSwitches, [](Packet& packet) {
        // uint8_t pressed = packet.read<uint8_t>();
        std::clog << packet.read<double>() << "\n";

        // check for which limit switches pressed
        // if (pressed & Constants::LimitSwitch::LEFT_PRESSED) {
        //     // left was pressed, decide what movements invalid
        //     std::clog << "Left Pressed\n";
        // }
        // if (pressed & Constants::LimitSwitch::RIGHT_PRESSED) {
        //     // right was pressed, decide what movements invalid
        //     std::clog << "Right Pressed\n";
        // }
        // if (pressed & Constants::LimitSwitch::BOTTOM_PRESSED) {
        //     // bottom was pressed, decide what movements invalid
        //     // std::clog << "Bottom Pressed\n";
        //     // std::clog <<  packet.read<Point2<double>>() << "\n";
        // }
        // if (pressed & Constants::LimitSwitch::TOP_PRESSED) {
        //     // top was pressed, decide what movements invalid
        //     std::clog << "Top Pressed\n";
        // }
    });
    SerialLink::registerHandler(Action::DistanceSensorRead, [](Packet& packet) {
        std::clog << "Updated position with distance sensors.\n";
    });
    SerialLink::registerHandler(Action::MalletHome, [](Packet& packet) {
        std::clog << "Bad distance sensor read.\n";
    });
}

void Table::updateTracker() {
    if (_tracker) {
        _tracker->track();

        if (_show_render)
            _tracker->display();
    }
}

void Table::updateRoutine() {
    if (_routine) {
        _routine->updateTarget();
        _routine->transmitTarget();
    }
}

const IObjectView& Table::puck() {
    // Return read-only reference to the puck
    return _puck;
}

const IObjectView& Table::mallet() {
    // Return read-only reference to the mallet
    return _mallet;
}

const Routine* Table::routine() {
    // Return read-only reference to the routine
    return _routine.get();
}

const PuckTracker* Table::tracker() {
    // Return read-only reference to the routine
    return _tracker.get();
}

void Table::showRender() {
    _show_render = true;
}

void Table::hideRender() {
    _show_render = false;
}
