#pragma once

#include <opencv2/opencv.hpp>
#include <functional>
#include <string>

class Button {
    protected:
        // Button position
        cv::Point _position;

        // Button size
        cv::Size _size;

        // Button color
        cv::Scalar _color;

        // Button text
        std::string _text;

        // Button ativation state
        bool _activated = false;

    public:
        /**
         * @brief Create a new button
         * 
         * @param position  The position of the button
         * @param size      The size of the button
         * @param color     The color of the button
         * @param text      The text of the button
         */
        Button(const cv::Point& position, const cv::Point& size, const cv::Scalar& color, const std::string& text);

        /**
         * @brief Destroy a button
         */
        virtual ~Button() = default;

        /**
         * @brief Display the button onto a window
         * 
         * @param window    The window to be displayed on
         */
        virtual void display(cv::Mat& window);

        /**
         * @brief Activate the button
         */
        virtual void activate();

        /**
         * @brief Deactivate the button
         */
        virtual void deactivate();

        /**
         * @brief Get whether the button is activated or not
         * 
         * @return  Whether the button is activated or not
         */
        bool isActivated() const;

        /**
         * @brief Get the button position
         * 
         * @return  The button position
         */
        cv::Point getPosition() const;

        /**
         * @brief Get the button size
         * 
         * @return  The button size
         */
        cv::Point getSize() const;

        /**
         * @brief Get the button color
         * 
         * @return  The button color
         */
        cv::Scalar getColor() const;

        /**
         * @brief Get the button text
         * 
         * @return  The button text
         */
        std::string getText() const;
};