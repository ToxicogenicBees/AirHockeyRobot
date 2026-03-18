#ifndef RAY3_HPP
#define RAY3_HPP

#include "Types/Point3.hpp"

template <class T>
class Ray3 {
    private:
        Point3<T> _position;
        Point3<T> _direction;

    public:
        /**
         * @brief   Create a new ray
         * 
         * @param   position  The origin of the ray
         * @param   direction The direction of the ray
         */
        Ray3(const Point3<T>& position, const Point3<T> direction);

        /**
         * @brief   Create a new ray positioned at the origin
         * 
         * @param   direction The direction of the ray
         */
        Ray3(const Point3<T>& direction);

        /**
         * @brief   Create a new ray positioned at the origin with no direction
         */
        Ray3();

        /**
         * @brief   Set the direction of the ray
         * 
         * @param   direction The new ray direction
         */
        void setDirection(const Point3<T>& direction);

        /**
         * @brief   Set the position of the ray
         * 
         * @param   direction The new ray position
         */
        void setPosition(const Point3<T>& position);

        /**
         * @brief   Get the ray direction
         * 
         * @return  The ray direction
         */
        Point3<T> direction() const;

        /**
         * @brief   Get the ray position
         * 
         * @return  The ray position
         */
        Point3<T> position() const;

        /**
         * @brief   Get this ray as a unit ray
         * 
         * @return  A unit ray positioned and pointing
         *          the same way as this ray
         */
        Ray3<T> unit() const;

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
        Point3<T> endPoint() const;
};

template <class T>
Ray3<T>::Ray3(const Point3<T>& position, const Point3<T> direction) {
    _position = position;
    _direction = direction;
}

template <class T>
Ray3<T>::Ray3(const Point3<T>& direction) {
    _position = Point3<T>::zero();
    _direction = direction;
}

template <class T>
Ray3<T>::Ray3() {
    _position = Point3<T>::zero();
    _direction = Point3<T>::zero();
}

template <class T>
void Ray3<T>::setDirection(const Point3<T>& direction) {
    _direction = direction;
}

template <class T>
void Ray3<T>::setPosition(const Point3<T>& position) {
    _position = position;
}

template <class T>
Point3<T> Ray3<T>::direction() const {
    return _direction;
}

template <class T>
Point3<T> Ray3<T>::position() const {
    return _position;
}

template <class T>
Ray3<T> Ray3<T>::unit() const {
    return Ray3{_position, _direction.normal()};
}

template <class T>
T Ray3<T>::magnitude() const {
    return _direction.magnitude();
}

template <class T>
T Ray3<T>::squaredMagnitude() const {
    return _direction.squaredMagnitude();
}

template <class T>
Point3<T> Ray3<T>::endPoint() const {
    return _position + _direction;
}

#endif
