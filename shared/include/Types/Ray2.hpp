#pragma once

#include "Types/Point2.hpp"

template <class T>
struct Ray2 {
    Point2<T> position;
    Point2<T> direction;

    /**
     * @brief   Create a new ray
     * 
     * @param   position  The origin of the ray
     * @param   direction The direction of the ray
     */
    Ray2(const Point2<T>& position, const Point2<T> direction);

    /**
     * @brief   Create a new ray positioned at the origin
     * 
     * @param   direction The direction of the ray
     */
    Ray2(const Point2<T>& direction);

    /**
     * @brief   Create a new ray positioned at the origin with no direction
     */
    Ray2() = default;

    /**
     * @brief   Get this ray as a unit ray
     * 
     * @return  A unit ray positioned and pointing
     *          the same way as this ray
     */
    Ray2<T> unit() const;

    /**
     * @brief   Get this ray's magnitude
     * 
     * @return  This ray's magnitude
     */
    T magnitude() const;

    /**
     * @brief   Get this ray's squared magnitude
     * 
     * @return  This ray's squared magnitude
     */
    T squaredMagnitude() const;

    /**
     * @brief   Get this ray's end point
     * 
     * @return  This ray's end point
     */
    Point2<T> endPoint() const;
};

template <class T>
Ray2<T>::Ray2(const Point2<T>& position, const Point2<T> direction) 
    : position(position), direction(direction) {}

template <class T>
Ray2<T>::Ray2(const Point2<T>& direction)
    : direction(direction) {}

template <class T>
Ray2<T> Ray2<T>::unit() const {
    return Ray2{position, direction.normal()};
}

template <class T>
T Ray2<T>::magnitude() const {
    return direction.magnitude();
}

template <class T>
T Ray2<T>::squaredMagnitude() const {
    return direction.squaredMagnitude();
}

template <class T>
Point2<T> Ray2<T>::endPoint() const {
    return position + direction;
}
