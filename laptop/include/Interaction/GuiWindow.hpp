#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

#include "Interaction/Button.hpp"

class GuiWindow {
    private:
        // Window name
        const std::string _NAME;

        // Window size
        const cv::Size _SIZE;

        // Buttons
        std::vector<Button*> _buttons;

        // Window
        cv::Mat _window;

    public:
        /**
         * @brief Create a new gui window
         * 
         * @param name  The window name
         */
        GuiWindow(const std::string& name, const cv::Size& size);

        /**
         * @brief Destroy a gui window
         */
        ~GuiWindow();

        /**
         * @brief Fetches user's mouse input and updates buttons accordingly
         */
        void fetchUserInput();

        /**
         * @brief Draw the current UI state to the window
         */
        void draw();

        /**
         * @brief Add a button to the window
         * 
         * @param position  The position of the button
         * @param size      The size of the button
         * @param color     The color of the button
         * @param text      The text of the button
         */
        template<class ButtonType>
        void addButton(const cv::Point& position, const cv::Point& size, const cv::Scalar& color, const std::string& text) {
            _buttons.push_back(new ButtonType(position, size, color, text));
        }

        /**
         * @brief Get the name of this window
         * 
         * @return  The name of the window
         */
        std::string getName() const;

        /**
         * @brief Get the size of this window
         * 
         * @return  The size of the window
         */
        cv::Size getSize() const;
};
