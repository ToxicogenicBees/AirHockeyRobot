#pragma once

#include <utility>
#include <vector>
#include <mutex>

#include "Motion/IObjectView.hpp"
#include "Types/Point2.hpp"
#include "Types/Timer.hpp"
#include "Types/Ray2.hpp"
#include "Constants.hpp"

class MovingObject : public IObjectView {
    private:
        const double _RADIUS;                       // Object radius, in inches

        mutable std::mutex _access_locational_data; // Mutex lock to guard access to position/velocity data
        Ray2<double> _orientation;                  // Position and velocity of the object
        Timer _timer;                               // Microsecond timer

    public:
        /**
         * @brief Create a new moving object
         * 
         * @param radius    The radius of the object
         */
        MovingObject(double radius);

        /**
         * @brief Prevent reassignment of moving objects
         */
        MovingObject& operator=(const MovingObject&) = delete;

        /**
         * @brief Updates the internal state of the object, traveling to the new position over the given time range
         * 
         * @param new_pos   The position the object is at now
         * @param dt        The time difference, in seconds, between position updates
         */
        void moveTo(const Point2<double>& new_pos, double dt);

        /**
         * @brief Updates the internal state of the object, traveling to the new position over the time since
         *        the object was last moved
         * 
         * @param new_pos   The position the object is at now
         */
        void moveTo(const Point2<double>& new_pos);

        /**
         * @brief Sets the position and velocity of the object
         * 
         * @param orientation   The desired orientation of the object
         */
        void orient(const Ray2<double>& orientation);

        /**
         * @brief Returns the current orientation of the object, in inches and inches/sec
         * 
         * @return The orientation of the object, in inches and inches/sec
         */
        Ray2<double> orientation() const override;

        /**
         * @brief Returns the future orientation of the object, in inches and inches/sec
         * 
         * @param dt    The change in time, in seconds
         * 
         * @return The future orientation of the object, in inches and inches/sec
         */
        Ray2<double> futureOrientation(double dt) const;

        /**
         * @brief Get the trajectory of the puck
         * 
         * @param include_return    Toggle whether the object's return to the human's
         *                          side of the table is included
         */
        std::vector<Timestamp> trajectory(bool include_return = false) const override;

        /**
         * @brief Returns the current position of the object, in inches
         * 
         * @return The position of the object, in inches
         */
        Point2<double> position() const override;

        /**
         * @brief Returns the current velocity of the object, in inches/sec
         * 
         * @return The velocity of the object, in inches/sec
         */
        Point2<double> velocity() const override;

        /**
         * 
         * @return The radius of the object, in inches
         */
        double radius() const override;
};
