#pragma once

#include "Interaction/Button.hpp"
#include "Motion/Table.hpp"

template <class RoutineType>
class RoutineButton : public Button {
    public:
        /**
         * @brief Create a new button
         * 
         * @param position  The position of the button
         * @param size      The size of the button
         * @param color     The color of the button
         * @param text      The text of the button
         */
        RoutineButton(const cv::Point& position, const cv::Point& size, const cv::Scalar& color, const std::string& text)
            : Button(position, size, color, text) {}
        
        /**
         * @brief Activate the button
         */
        void activate() final {
            Button::activate();
            Table::setRoutine<RoutineType>();
        }
};