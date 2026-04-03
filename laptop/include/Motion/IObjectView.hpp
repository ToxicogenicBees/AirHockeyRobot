#pragma once

#include <utility>
#include <vector>

#include "Types/Point2.hpp"
#include "Types/Ray2.hpp"

class IObjectView {
    protected:
        using Timestamp = std::pair<double, Ray2<double>>;

    public:
        /**
         * @brief Destroy an IObjectView
         */
        virtual ~IObjectView() = default;

        /**
         * @brief Get the trajectory of the objecy
         * 
         * @param include_return    Toggle whether the object's return to the human's
         *                          side of the table is included
         */
        virtual std::vector<Timestamp> trajectory(bool include_return = false) const = 0;

        /**
         * @brief Get the object's position, in inches
         * 
         * @return The object's position, in inches
         */
        virtual Point2<double> position() const = 0;

        /**
         * @brief Get the object's velocity, in inches/sec
         * 
         * @return The object's velocity, in inches/sec
         */
        virtual Point2<double> velocity() const = 0;

        /**
         * @brief Get the radius of the object, in inches
         * 
         * @return The radius of the object, in inches
         */
        virtual double radius() const = 0;
};