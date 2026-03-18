#ifndef MALLET_H
#define MALLET_H

#include <vector>
#include <memory>

#include "Motion/Routines/Routine.h"
#include "Motion/MovingObject.h"
#include "Types/Point2.hpp"
#include "Types/Point3.hpp"
#include "Types/Ray2.hpp"
#include "Constants.h"

class Mallet {
    private:
        static MovingObject _mallet;                // Moving object used for the mallet
        static std::unique_ptr<Routine> _routine;   // Current mallet routine

    public:
        /**
         * @brief Updates the internal state of the object, traveling to the new position over the given time range
         * 
         * @param new_pos   The position the object is at now
         * @param micsec    The time difference in microseconds between position updates,
         *                  defaults to the time between function calls
         */
        static void moveTo(const Point2<double>& new_pos, int64_t micsec = -1);

        /**
         * @brief Sets the position and velocity of the object
         * 
         * @param orientation   The desired orientation of the object
         */
        static void orient(const Ray2<double>& orientation);

        /**
         * @brief Sets the mallet routine
         * 
         * @param routine The desired mallet routine
         */
        static void setRoutine(std::unique_ptr<Routine> routine);

        /**
         * @brief Calculates an appropriate mallet action from the mallet's routine
         */
        static void updateTarget();

        /**
         * @brief Transmits the target mallet action over the SerialLink
         */
        static void transmitTarget();

        /**
         * @brief Returns the current position of the object, in inches and inches/sec
         * 
         * @return The orientation of the object, in inches and inches/sec
         */
        Ray2<double> orientation();

        /**
         * @brief Returns the current position of the object, in inches
         * 
         * @return The position of the object, in inches
         */
        static Point2<double> position();

        /**
         * @brief Returns the current velocity of the object, in inches/sec
         * 
         * @return The velocity of the object, in inches/sec
         */
        static Point2<double> velocity();

        /**
         * @brief Returns the current target chosen
         * 
         * @return The current target chosen
         */
        static Point2<double> target();
};

#endif
