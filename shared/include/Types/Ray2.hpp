#ifndef RAY2_HPP
#define RAY2_HPP

#include "Types/Point2.hpp"

template <class T>
class Ray2 {
    private:
        Point2<T> _position;
        Point2<T> _direction;

    public:
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
        Ray2();

        /**
         * @brief   Set the direction of the ray
         * 
         * @param   direction The new ray direction
         */
        void setDirection(const Point2<T>& direction);

        /**
         * @brief   Set the position of the ray
         * 
         * @param   direction The new ray position
         */
        void setPosition(const Point2<T>& position);

        /**
         * @brief   Get the ray direction
         * 
         * @return  The ray direction
         */
        Point2<T> direction() const;

        /**
         * @brief   Get the ray position
         * 
         * @return  The ray position
         */
        Point2<T> position() const;

        /**
         * @brief   Get this ray as a unit array
         * 
         * @return  A unit array positioned and pointing
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
Ray2<T>::Ray2(const Point2<T>& position, const Point2<T> direction) {
    _position = position;
    _direction = direction;
}

template <class T>
Ray2<T>::Ray2(const Point2<T>& direction) {
    _position = Point2<T>::zero();
    _direction = direction;
}

template <class T>
Ray2<T>::Ray2() {
    _position = Point2<T>::zero();
    _direction = Point2<T>::zero();
}

template <class T>
void Ray2<T>::setDirection(const Point2<T>& direction) {
    _direction = direction;
}

template <class T>
void Ray2<T>::setPosition(const Point2<T>& position) {
    _position = position;
}

template <class T>
Point2<T> Ray2<T>::direction() const {
    return _direction;
}

template <class T>
Point2<T> Ray2<T>::position() const {
    return _position;
}

template <class T>
Ray2<T> Ray2<T>::unit() const {
    return Ray2{_position, _direction.normal()};
}

template <class T>
T Ray2<T>::magnitude() const {
    return _direction.magnitude();
}

template <class T>
T Ray2<T>::squaredMagnitude() const {
    return _direction.squaredMagnitude();
}

template <class T>
Point2<T> Ray2<T>::endPoint() const {
    return _position + _direction;
}

#endif
