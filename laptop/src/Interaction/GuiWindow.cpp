#include <opencv2/opencv.hpp>
#include <windows.h>

#include "Interaction/GuiWindow.hpp"

namespace {
    bool mouse1Down() {
        bool state = GetAsyncKeyState(VK_LBUTTON) & 0x0001;
        static bool held = false;

        if (!held && state) {
            held = true;
            return true;
        }
        else if (held && !state) {
            held = false;
        }

        return false;
    }

    POINT mousePos() {
        POINT p;
        GetCursorPos(&p);
        return p;
    }
};

GuiWindow::GuiWindow(const std::string& name, const cv::Size& size)
    : _NAME(name), _SIZE(size), _window(cv::Mat::zeros(_SIZE.height, _SIZE.width, CV_8UC3)) {}

GuiWindow::~GuiWindow() {
    for (auto b : _buttons)
        delete b;
}

void GuiWindow::fetchUserInput() {
    // Ignore if mouse isn't pressed
    if (!mouse1Down())
        return;

    // Fetch mouse position
    auto mouse_pos = mousePos();

    // Get mouse position on screen
    auto hwnd = FindWindowA(NULL, _NAME.c_str());
    ScreenToClient(hwnd, &mouse_pos);

    // Check if client is hovered over a button
    for (auto b : _buttons) {
        auto b_tl = b->getPosition();
        auto b_br = b_tl + b->getSize();

        if (b_tl.x <= mouse_pos.x && b_tl.y <= mouse_pos.y && mouse_pos.x <= b_br.x && mouse_pos.y <= b_br.y)
            b->activate();
        else
            b->deactivate();
    }

    // Update the window
    draw();
}

void GuiWindow::draw() {
    _window.setTo(cv::Scalar(0, 0, 0));

    for (auto b : _buttons)
        b->display(_window);
    
    cv::imshow(_NAME, _window);
    cv::waitKey(1);
}

std::string GuiWindow::getName() const {
    return _NAME;
}

cv::Size GuiWindow::getSize() const {
    return _SIZE;
}
