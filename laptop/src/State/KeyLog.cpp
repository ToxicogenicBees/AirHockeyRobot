#include "State/KeyLog.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>

std::map<char, std::function<void()>> KeyLog::_actions;
std::thread KeyLog::_action_thread;
std::mutex KeyLog::_action_guard;

void KeyLog::_actOnInput() {
    while (1) {
        // Wait for the next user input
        std::clog << "Logging\n";
        char key;
        std::cin >> key;

        // Run function bound to this key
        std::lock_guard<std::mutex> guard(_action_guard);
        auto act_iter = _actions.find(key);
        if (act_iter != _actions.end()) {
            std::clog << "Logged " << key << "\n";
            act_iter->second();
        }
    }
}

void KeyLog::init() {
    _action_thread = std::thread(_actOnInput);
    _action_thread.detach();
}

void KeyLog::connect(char key, std::function<void()> func) {
    std::lock_guard<std::mutex> guard(_action_guard);
    _actions[key] = func;
}

void KeyLog::disconnect(char key) {
    std::lock_guard<std::mutex> guard(_action_guard);
    _actions.erase(key);
}