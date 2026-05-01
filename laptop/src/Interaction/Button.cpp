#include "Interaction/Button.hpp"

Button::Button(const cv::Point& position, const cv::Point& size, const cv::Scalar& color, const std::string& text)
    : _position(position), _size(size), _color(color), _text(text) {}

void Button::display(cv::Mat& window) {
    // Draw button rectangle
    cv::rectangle(window, _position, {_position.x + _size.width, _position.y + _size.height}, _color, -1);

    // Determine text size
    int baseline;
    auto text_size = cv::getTextSize(_text, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
    
    double height_scale = (double)(_size.height) / text_size.height;
    double width_scale = (double)(_size.width) / text_size.width;
    auto scale = std::min(width_scale, height_scale);
    
    // Draw text
    auto text_origin = _position + cv::Point{0, _size.height};
    cv::putText(window, _text, text_origin, cv::FONT_HERSHEY_PLAIN, scale, {0, 0, 0}, 3);
}

void Button::activate() {
    _activated = true;
    _color = cv::Scalar(0, 255, 0);
}

void Button::deactivate() {
    _activated = false;
    _color = cv::Scalar(255, 255, 255);
}

bool Button::isActivated() const {
    return _activated;
}

cv::Point Button::getPosition() const {
    return _position;
}

cv::Point Button::getSize() const {
    return _size;
}

cv::Scalar Button::getColor() const {
    return _color;
}

std::string Button::getText() const {
    return _text;
}
