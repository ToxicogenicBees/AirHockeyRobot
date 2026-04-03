#pragma once

#include <memory>

#include "Tracking/CameraTracker.hpp"
#include "Motion/MovingObject.hpp"
#include "Motion/IObjectView.hpp"
#include "Routines/Routine.hpp"

class Table {
    private:
        // Moving objects
        static MovingObject _mallet;
        static MovingObject _puck;

        // Control
        static std::unique_ptr<PuckTracker> _tracker;
        static std::unique_ptr<Routine> _routine;
        inline static bool _show_render = true;

    public:
        /**
         * @brief Initialize the table and its components
         */
        static void init();

        /**
         * @brief Dynamically assign a new target routine
         * 
         * @param ... Constructor arguments for the routine type provided
         */
        template <typename RoutineType, typename... Args>
        static void setRoutine(Args&&... args) {
            _routine = std::make_unique<RoutineType>(_mallet, std::forward<Args>(args)...);
        }

        /**
         * @brief Dynamically assign a new puck tracker
         * 
         * @param ... Constructor arguments for the tracker type provided
         */
        template <typename TrackerType, typename... Args>
        static void setTracker(Args&&... args) {
            _tracker = std::make_unique<TrackerType>(_puck, std::forward<Args>(args)...);
            _tracker->init();
        }

        /**
         * @brief Run a track + display cycle for the puck tracker
         */
        static void updateTracker();

        /**
         * @brief Run a routine update
         */
        static void updateRoutine();

        /**
         * @brief Get read-only view of the puck
         * 
         * @return A read-only view of the puck
         */
        static const IObjectView& puck();

        /**
         * @brief Get read-only view of the mallet
         * 
         * @return A read-only view of the mallet
         */
        static const IObjectView& mallet();

        /**
         * @brief Get read-only view of the routine
         * 
         * @return A read-only view of the routine
         */
        static const Routine* routine();

        /**
         * @brief Get read-only view of the tracker
         * 
         * @return A read-only view of the tracker
         */
        static const PuckTracker* tracker();

        /**
         * @brief Enable rendering visuals
         */
        static void showRender();

        /**
         * @brief Disable rendering visuals
         */
        static void hideRender();
};
