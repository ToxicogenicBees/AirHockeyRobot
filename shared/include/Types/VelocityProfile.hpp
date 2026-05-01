#pragma once

#include <stdint.h>
#include <limits>

class VelocityProfile {
    private:
        uint8_t _accel_percent;
        uint8_t _decel_percent;
        uint16_t _min_rpm;
        uint16_t _max_rpm;

    public:
        /**
         * @brief Create a new velocity profile
         * 
         * @param accel_percent The acceleration percent value from 0 to 1.
         * @param decel_percent The deceleration percent value from 0 to 1.
         * @param min_rpm       The minimum RPM
         * @param max_rpm       The maximum RPM
         */
        VelocityProfile(double accel_percent, double decel_percent, uint16_t min_rpm, uint16_t max_rpm) {
            _accel_percent = (uint8_t) (accel_percent * std::numeric_limits<uint8_t>::max());
            _decel_percent = (uint8_t) (decel_percent * std::numeric_limits<uint8_t>::max());
            _min_rpm = min_rpm;
            _max_rpm = max_rpm;
        }

        /**
         * @brief Create a new velocity profile
         */
        VelocityProfile() = default;

        /**
         * @brief Returns the acceleration percent
         * 
         * @return The acceleration percent
         */
        double getAccelPercent() const {
            return (double)(_accel_percent) / std::numeric_limits<uint8_t>::max();
        }

        /**
         * @brief Returns the deceleration percent
         * 
         * @return The deceleration percent
         */
        double getDecelPercent() const {
            return (double)(_decel_percent) / std::numeric_limits<uint8_t>::max();
        }

        /**
         * @brief Returns the min RPM
         * 
         * @return The min RPM
         */
        uint16_t getMinRPM() const {
            return _min_rpm;
        }

        /**
         * @brief Returns the max RPM
         * 
         * @return The max RPM
         */
        uint16_t getMaxRPM() const {
            return _max_rpm;
        }

        /**
         * @brief Sets the acceleration percent
         * 
         * @param accel_percent The acceleration percent
         */
        template <class T>
        void setAccelPercent(T accel_percent) {
            _accel_percent = (uint8_t) accel_percent;
        }

        /**
         * @brief Sets the deceleration percent
         * 
         * @param decel_percent The deceleration percent
         */
        template <class T>
        void setDecelPercent(T decel_percent) {
            _decel_percent = (uint8_t) decel_percent;
        }

        /**
         * @brief Sets the min RPM
         * 
         * @param min_rpm The min RPM
         */
        template <class T>
        void setMinRPM(T min_rpm) {
            _min_rpm = (uint16_t) min_rpm;
        }

        /**
         * @brief Sets the max RPM
         * 
         * @param max_rpm The max RPM
         */
        template <class T>
        void setMaxRPM(T max_rpm) {
            _max_rpm = (uint16_t) max_rpm;
        }
};
